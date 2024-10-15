#include "nomadz_communication/udp_communicator.hpp"

#include <string>
#include <chrono>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "nomadz_communication/ros_conversion.hpp"
#include "nomadz_communication/team_comm_data.hpp"
#include "nomadz_communication_msgs/msg/team_comm_data.hpp"
#include "nomadz_communication_msgs/robocup_game_control_data_enums.hpp"
#include "nomadz_configuration/game_settings.hpp"
#include "nomadz_configuration/io.hpp"
#include "nomadz_proprioception_msgs/fall_down_state_enums.hpp"

namespace fs = std::filesystem;
using boost::asio::ip::udp;
using namespace std::chrono_literals;
namespace nomadz_communication {
  UDPCommunicator::UDPCommunicator(const rclcpp::NodeOptions& options)
      : Node("udp_communicator", options), game_settings_(nomadz_configuration::getGameSettings()),
        gc_socket_(*io_context_, udp::endpoint(udp::v4(), GAMECONTROLLER_DATA_PORT)),
        team_comm_data_socket_(*io_context_, udp::endpoint(udp::v4(), 10000 + game_settings_.team_id)) {

    initGCReturnData();
    team_comm_data_socket_.set_option(boost::asio::socket_base::reuse_address(true));
    team_comm_data_socket_.set_option(boost::asio::socket_base::broadcast(true));
    last_team_comm_send_time_ = this->now();

    // Note: udp::endpoint(boost::asio::ip::address_v4::broadcast(), 10000 + game_settings_.team_id); is not SPL enough
    team_comm_broadcast_addr_ =
      udp::endpoint(boost::asio::ip::address_v4::from_string("10.0.255.255"), 10000 + game_settings_.team_id);
    startGCReceive();
    startTeamCommReceive();

    gc_data_publisher_ = create_publisher<RobocupGameControlDataMsgT>(GC_DATA_TOPIC, 10);
    team_comm_info_publisher_ = create_publisher<TeamCommInfoMsgT>(TEAM_COMM_INFO_TOPIC, 10);

    fall_down_state_sub_ =
      create_subscription<FallDownStateMsgT>(FALL_DOWN_STATE_TOPIC, 1, [this](FallDownStateMsgT::ConstSharedPtr msg) {
        return_data_.has_fallen =
          msg->fall_down_state == static_cast<uint8_t>(nomadz_proprioception_msgs::FallDownState::ON_GROUND);
      });

    ego_status_sub_ = create_subscription<EgoStatusMsgT>(
      EGO_STATUS_TOPIC, 1, [this](EgoStatusMsgT::ConstSharedPtr msg) { ego_status_ = *msg; });
    world_model_sub_ = create_subscription<WorldModelMsgT>(WORLD_MODEL_TOPIC, 1, [this](WorldModelMsgT::ConstSharedPtr msg) {
      world_model_ = *msg;
      return_data_.pose[0] = static_cast<float>(msg->robot_pose.pose.x) * 1000.F;
      return_data_.pose[1] = static_cast<float>(msg->robot_pose.pose.y) * 1000.F;
      return_data_.pose[2] = static_cast<float>(msg->robot_pose.pose.theta);
      return_data_.ball[0] = static_cast<float>(msg->ball_model.position.x) * 1000.F;
      return_data_.ball[1] = static_cast<float>(msg->ball_model.position.y) * 1000.F;
      return_data_.ball_age =
        static_cast<float>((static_cast<rclcpp::Duration>(this->now() - msg->ball_model.last_valid_time_stamp)).seconds());
    });

    timer_ = create_wall_timer(500ms, [this]() {
      if (sendThisFrame()) {
        teamCommCallback();
      }
    });

    io_thread_ = std::thread([this]() { io_context_->run(); });
  }

  UDPCommunicator::~UDPCommunicator() {
    io_context_->stop();
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
    RCLCPP_DEBUG(get_logger(), "Shutting down UDPCommunicator");
  }

  void UDPCommunicator::initGCReturnData() {
    std::string header = GAMECONTROLLER_RETURN_STRUCT_HEADER;
    header.copy(return_data_.header, header.size());
    return_data_.version = GAMECONTROLLER_RETURN_STRUCT_VERSION;
    return_data_.player_num = game_settings_.player_id;
    return_data_.team_num = game_settings_.team_id;
    return_data_.has_fallen = static_cast<uint8_t>(false);
    return_data_.pose[0] = 0.F;
    return_data_.pose[1] = 0.F;
    return_data_.pose[2] = 0.F;
    return_data_.ball_age = 0.F;
    return_data_.ball[0] = 0.F;
    return_data_.ball[1] = 0.F;
  }

  // GAME CONTROL
  void UDPCommunicator::startGCReceive() {
    gc_socket_.async_receive_from(boost::asio::buffer(gc_recv_buffer_),
                                  gc_server_sender_addr_,
                                  [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
                                    handleGCReceive(error, bytes_transferred);
                                  });
  }

  void UDPCommunicator::handleGCReceive(const boost::system::error_code& error, std::size_t bytes_transferred) {
    if (!error || error == boost::asio::error::message_size) {
      gc_data_ = *(reinterpret_cast<const RobocupGameControlData*>(&gc_recv_buffer_));
      processGCData();
      returnDataCallback();
      gc_data_publisher_->publish(packRobocupGameControlData(RobocupGameControlData(gc_data_), this->now()));
      if (!gc_host_addr_set_) {
        gc_host_addr_set_ = true;
        gc_server_receiver_addr_ = gc_server_sender_addr_;
        gc_server_receiver_addr_.port(GAMECONTROLLER_RETURN_PORT);
      }

      RCLCPP_DEBUG(get_logger(), "Received %d bytes of data from GC", static_cast<int>(bytes_transferred));
      startGCReceive();

    } else {
      RCLCPP_ERROR(get_logger(), "Failed to receive data from GC");
    }
  }

  void UDPCommunicator::returnDataCallback() {
    if (gc_host_addr_set_) {
      const size_t return_data_buffer_size = sizeof(return_data_);
      char return_data_buffer[return_data_buffer_size + 1];
      std::memcpy(return_data_buffer, &return_data_, return_data_buffer_size);
      return_data_buffer[return_data_buffer_size] = '\0';

      gc_socket_.async_send_to(boost::asio::buffer(return_data_buffer),
                               gc_server_receiver_addr_,
                               [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
                                 handleGCSend(error, bytes_transferred);
                               });
    }
  }

  void UDPCommunicator::handleGCSend(const boost::system::error_code& error, std::size_t bytes_transferred) {
    if (!error || error == boost::asio::error::message_size) {
      RCLCPP_DEBUG(get_logger(), "Sent %d bytes of return data to GameController", static_cast<int>(bytes_transferred));
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to send return data to GameController");
    }
  }

  // TEAM COMM
  void UDPCommunicator::startTeamCommReceive() {
    team_comm_data_socket_.async_receive_from(boost::asio::buffer(team_comm_recv_buffer_),
                                              team_comm_sender_addr_,
                                              [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
                                                handleTeamCommReceive(error, bytes_transferred);
                                              });
  }

  void UDPCommunicator::handleTeamCommReceive(const boost::system::error_code& error, std::size_t bytes_transferred) {
    if (!error || error == boost::asio::error::message_size) {
      const TeamCommData team_comm_data = *(reinterpret_cast<const TeamCommData*>(&team_comm_recv_buffer_));
      nomadz_communication_msgs::msg::TeamCommData team_comm_data_msgs = packTeamCommData(team_comm_data, this->now());
      team_comm_info_.team_comm_info[team_comm_data_msgs.player_id - 1] = team_comm_data_msgs;
      team_comm_info_.header.stamp = this->now();
      team_comm_info_publisher_->publish(team_comm_info_);

      RCLCPP_DEBUG(get_logger(), "Received %d bytes of Team Comm Data from GC", static_cast<int>(bytes_transferred));
      startTeamCommReceive();

    } else {
      RCLCPP_ERROR(get_logger(), "Failed to receive Team Comm Data from GC");
    }
  }

  void UDPCommunicator::teamCommCallback() {
    if (gc_host_addr_set_) {
      TeamCommData team_comm_data = unpackTeamCommData(game_settings_.player_id, ego_status_, world_model_);
      const size_t buffer_size = sizeof(team_comm_data);
      char team_comm_data_buffer[buffer_size + 1];
      std::memcpy(team_comm_data_buffer, &team_comm_data, buffer_size);
      team_comm_data_buffer[buffer_size] = '\0';

      static_assert(buffer_size + 1 <= MAX_TEAM_COMM_DATA_SIZE);

      team_comm_data_socket_.async_send_to(boost::asio::buffer(team_comm_data_buffer),
                                           team_comm_broadcast_addr_,
                                           [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
                                             handleTeamCommSend(error, bytes_transferred);
                                           });
    }
  }

  void UDPCommunicator::handleTeamCommSend(const boost::system::error_code& error, std::size_t bytes_transferred) {
    if (error == boost::asio::error::network_unreachable) {
      RCLCPP_ERROR(get_logger(), "Network unreachable");
    }
    if (!error || error == boost::asio::error::message_size) {
      RCLCPP_DEBUG(get_logger(), "Sent %d bytes of Team Comm Data to GC", static_cast<int>(bytes_transferred));
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to send Team Comm Data to GC");
    }
  }

  void UDPCommunicator::processGCData() {
    for (const auto team_info : gc_data_.teams) {
      if (team_info.team_id == game_settings_.team_id) {
        own_team_info_ = team_info;
        break;
      }
    }
    own_robot_info_ = own_team_info_.players[game_settings_.player_id - 1];
  }

  bool UDPCommunicator::sendThisFrame() {
    if (own_team_info_.message_budget <= 15) {
      return false;
    }
    if (own_robot_info_.penalty != nomadz_communication_msgs::Penalty::NONE) {
      return false;
    }
    if (gc_data_.state != nomadz_communication_msgs::GameState::PLAYING) {
      return false;
    }
    // TODO(Zichong): Implement more when to send logic
    const float seconds_until_end =
      static_cast<float>(gc_data_.secs_remaining) + static_cast<float>(gc_data_.first_half) * 600.F;
    const float send_time_out = seconds_until_end / static_cast<float>(own_team_info_.message_budget) * 7.F;
    const float time_since_last_send =
      static_cast<float>((static_cast<rclcpp::Duration>(this->now() - last_team_comm_send_time_)).seconds());
    if (time_since_last_send < send_time_out) {
      return false;
    }
    last_team_comm_send_time_ = this->now();
    return true;
  }
} // namespace nomadz_communication

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nomadz_communication::UDPCommunicator)
