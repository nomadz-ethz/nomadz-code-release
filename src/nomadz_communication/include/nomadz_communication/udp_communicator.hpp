#pragma once

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "nomadz_configuration/game_settings.hpp"
#include "nomadz_communication/robocup_game_control_data.hpp"
#include "nomadz_communication/robocup_game_control_return_data.hpp"
#include "nomadz_communication_msgs/msg/robocup_game_control_data.hpp"
#include "nomadz_communication_msgs/msg/team_comm_info.hpp"
#include "nomadz_modeling_msgs/msg/world_model.hpp"
#include "nomadz_proprioception_msgs/msg/fall_down_state.hpp"

namespace nomadz_communication {
  constexpr int GAMECONTROLLER_DATA_PORT = 3838;
  constexpr int GAMECONTROLLER_RETURN_PORT = 3939;
  constexpr int MAX_TEAM_COMM_DATA_SIZE = 128;

  class UDPCommunicator : public rclcpp::Node {
    using RobocupGameControlDataMsgT = nomadz_communication_msgs::msg::RobocupGameControlData;
    using TeamCommInfoMsgT = nomadz_communication_msgs::msg::TeamCommInfo;
    using EgoStatusMsgT = nomadz_behavior_msgs::msg::EgoStatus;
    using FallDownStateMsgT = nomadz_proprioception_msgs::msg::FallDownState;
    using WorldModelMsgT = nomadz_modeling_msgs::msg::WorldModel;

    static constexpr const char* GC_DATA_TOPIC = "communication/gc_data";
    static constexpr const char* TEAM_COMM_INFO_TOPIC = "communication/team_comm_info";

    static constexpr const char* EGO_STATUS_TOPIC = "behavior/ego_status";
    static constexpr const char* WORLD_MODEL_TOPIC = "modeling/world_model";
    static constexpr const char* GC_RETURN_DATA_TOPIC = "communication/gc_return_data";
    static constexpr const char* FALL_DOWN_STATE_TOPIC = "proprioception/fall_down_state";

  public:
    explicit UDPCommunicator(const rclcpp::NodeOptions& options);
    ~UDPCommunicator() override;

  private:
    void initGCReturnData();

    // GAME CONTROL
    void startGCReceive();
    void handleGCReceive(const boost::system::error_code& error, std::size_t bytes_transferred);
    void returnDataCallback();
    void handleGCSend(const boost::system::error_code& error, std::size_t bytes_transferred);

    // TEAM COMM
    void startTeamCommReceive();
    void handleTeamCommReceive(const boost::system::error_code& error, std::size_t bytes_transferred);

    void teamCommCallback();
    void handleTeamCommSend(const boost::system::error_code& error, std::size_t bytes_transferred);

    bool sendThisFrame();
    void processGCData();

    static constexpr int BUFFER_LEN = 1024;

    std::shared_ptr<boost::asio::io_context> io_context_ = std::make_shared<boost::asio::io_context>();
    std::thread io_thread_;

    const nomadz_configuration::GameSettings game_settings_;

    bool gc_host_addr_set_ = false;
    boost::asio::ip::udp::socket gc_socket_;
    boost::asio::ip::udp::endpoint gc_server_sender_addr_;
    boost::asio::ip::udp::endpoint gc_server_receiver_addr_;
    std::array<char, BUFFER_LEN> gc_recv_buffer_;

    boost::asio::ip::udp::endpoint team_comm_broadcast_addr_;
    boost::asio::ip::udp::endpoint team_comm_sender_addr_;
    boost::asio::ip::udp::socket team_comm_data_socket_;
    std::array<char, BUFFER_LEN> team_comm_recv_buffer_;

    EgoStatusMsgT ego_status_;
    WorldModelMsgT world_model_;
    TeamCommInfoMsgT team_comm_info_;

    RobocupGameControlData gc_data_;
    RobocupGameControlReturnData return_data_;
    TeamInfo own_team_info_;
    RobotInfo own_robot_info_;

    std::shared_ptr<rclcpp::TimerBase> timer_;
    rclcpp::Time last_team_comm_send_time_;

    rclcpp::Publisher<RobocupGameControlDataMsgT>::SharedPtr gc_data_publisher_;
    rclcpp::Publisher<TeamCommInfoMsgT>::SharedPtr team_comm_info_publisher_;

    rclcpp::Subscription<EgoStatusMsgT>::SharedPtr ego_status_sub_;
    rclcpp::Subscription<FallDownStateMsgT>::SharedPtr fall_down_state_sub_;
    rclcpp::Subscription<WorldModelMsgT>::SharedPtr world_model_sub_;
  };
} // namespace nomadz_communication
