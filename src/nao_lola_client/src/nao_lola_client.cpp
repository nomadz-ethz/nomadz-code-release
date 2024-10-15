// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nao_lola_client/nao_lola_client.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <string>
#include <stdexcept>

#include "nao_lola_client/conversion.hpp"
#include "nomadz_core/math/constants.hpp"

namespace nao_lola_client {

  NaoLolaClient::NaoLolaClient()
      : Node("NaoLolaClient"), parameters_(nao_lola_client::ParamListener(get_node_parameters_interface()).get_params()),
        time_when_chest_button_was_last_unpressed_(rclcpp::Clock{}.now()),
        time_when_last_emergency_state_switch_(rclcpp::Clock{}.now()) {

    Connection::endpoint_t endpoint;
    if (parameters_.endpoint.connection_type == "UNIX") {
      endpoint = boost::asio::local::stream_protocol::endpoint(parameters_.endpoint.nao_lola_unix_endpoint);
    } else if (parameters_.endpoint.connection_type == "TCP") {
      if (parameters_.endpoint.tcp_port < 0) {
        RCLCPP_FATAL(this->get_logger(),
                     "TCP endpoint requested but no valid port was provided, %ld. Exiting the LoLA client.",
                     parameters_.endpoint.tcp_port);
        throw std::invalid_argument(" Received an invalid port number");
      }
      endpoint = boost::asio::ip::tcp::endpoint{boost::asio::ip::address_v4::loopback(),
                                                static_cast<unsigned short>(parameters_.endpoint.tcp_port)};
    } else {
      RCLCPP_FATAL(this->get_logger(),
                   "Required endpoint of type %s does not match available types (UNIX, TCP).",
                   parameters_.endpoint.connection_type.c_str());
    }
    connection_.connect(endpoint);

    createPublishers();
    createSubscriptions();

    std_msgs::msg::Bool unstiff_state{};
    unstiff_state.data = true;
    unstiff_state_pub_->publish(unstiff_state);

    // Start receive and send loop
    receive_thread_ = std::thread([this]() {
      while (rclcpp::ok()) {
        auto recv_data = connection_.receive();
        rclcpp::Time time_stamp = this->now();
        const MsgpackParser parsed(recv_data.data(), recv_data.size(), time_stamp);
        publish(parsed);
        checkButtonStates(parsed);

        if (is_in_emergency_state_ && !is_seated_) {
          sitDown(parsed);
          unstiffed_ = true;
          is_seated_ = true;
        }
        if (!is_in_emergency_state_ && is_seated_) {
          standUp(parsed);
          unstiffed_ = false;
          is_seated_ = false;
        }

        // If there are no publishers to the joint position data, we sit down and
        // exit the threadloop.
        if (parameters_.shutdown_if_no_joint_requests_publishers) {
          if (count_publishers("effectors/joint_requests") == 0 || chest_button_shutdown_requested_) {
            RCLCPP_FATAL(this->get_logger(), "No publishers to the joint_requests topic. Shutting down the client!");
            sitDown(parsed);
            rclcpp::sleep_for(std::chrono::seconds(2));
            throw RosShutDownRequestException();
          }
        }
        sendData();
      }
    });
  }

  NaoLolaClient::~NaoLolaClient() {
    RCLCPP_DEBUG(get_logger(), "Shutting down NaoLolaClient");
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
  }

  void NaoLolaClient::sendData() {
    // In mutex, copy packer_
    // Do the pack and send outside mutex to avoid retain lock for a long time
    MsgpackPacker packer_copy;
    {
      const std::lock_guard<std::mutex> guard(packer_mutex_);
      packer_copy = packer_;
    }
    connection_.send(packer_copy.getPacked());
  }

  void NaoLolaClient::checkButtonStates(const MsgpackParser& parsed_sensor_data) {
    std::vector<float> buttons = parsed_sensor_data.getSensorData("Touch");
    const bool chest_button_is_pressed = buttons.at(static_cast<int>(LolaEnums::Touch::ChestBoard_Button)) != 0.0F;

    // If chest button is pressed and is pressed for long enough, we request a shutdown.
    if (!chest_button_is_pressed) {
      time_when_chest_button_was_last_unpressed_ = rclcpp::Clock{}.now();
    } else if ((rclcpp::Clock{}.now() - time_when_chest_button_was_last_unpressed_).seconds() >= 5.0) {
      chest_button_shutdown_requested_ = true;
    }

    unsigned int number_of_head_buttons_pressed = 0;
    number_of_head_buttons_pressed +=
      static_cast<unsigned int>(buttons.at(static_cast<int>(LolaEnums::Touch::Head_Touch_Front)));
    number_of_head_buttons_pressed +=
      static_cast<unsigned int>(buttons.at(static_cast<int>(LolaEnums::Touch::Head_Touch_Middle)));
    number_of_head_buttons_pressed +=
      static_cast<unsigned int>(buttons.at(static_cast<int>(LolaEnums::Touch::Head_Touch_Rear)));

    // If more than 2 head buttons are pressed and they are pressed long enough, we switch the emergency state.
    // Furthermore, we add a hysteresis time of 2s to prevent rapid switching of the state.
    const bool more_than_two_head_buttons_pressed = number_of_head_buttons_pressed > 2;
    if (!more_than_two_head_buttons_pressed) {
      time_when_all_head_buttons_was_last_unpressed_ = rclcpp::Clock{}.now();
    } else if ((rclcpp::Clock{}.now() - time_when_all_head_buttons_was_last_unpressed_).seconds() >= 1.0 &&
               (rclcpp::Clock{}.now() - time_when_last_emergency_state_switch_).seconds() >= 2.0) {
      time_when_last_emergency_state_switch_ = rclcpp::Clock{}.now();
      is_in_emergency_state_ = !is_in_emergency_state_;
      RCLCPP_WARN(this->get_logger(), "Emergency state changed to %s", is_in_emergency_state_ ? "true" : "false");
    }
  }

  void NaoLolaClient::sitDown(const MsgpackParser& parsed_sensor_data) {
    // Determine current angles and whether sitting down is required, i.e. has the hip stiffness?
    bool sit_down_required = false;
    std::vector<float> start_angles = parsed_sensor_data.getSensorData("Position");
    std::vector<float> const torso_angles = parsed_sensor_data.getSensorData("Angles");
    std::vector<float> stiffnesses = parsed_sensor_data.getSensorData("Stiffness");
    std::array<float, static_cast<int>(LolaEnums::Joint::NUM_JOINTS)> sit_down_angles;
    std::transform(parameters_.sit_down_angles.begin(),
                   parameters_.sit_down_angles.end(),
                   sit_down_angles.begin(),
                   [](double x) { return static_cast<float>(x); });

    if (stiffnesses[static_cast<int>(LolaEnums::Joint::LHipRoll)] > 0.0 ||
        stiffnesses[static_cast<int>(LolaEnums::Joint::RHipRoll)] > 0.0) {
      sit_down_required = true;
    }

    // If sitting down is required, interpolate from start angles to target angles.
    if (sit_down_required) {
      float ratio = 0.F;
      while (ratio < 1.F) {
        // Do sinus interpolation
        const float phase = 0.5F * std::sin((ratio - 0.5F) * nomadz_core::constants::PI) + 0.5F;

        auto recv_data = connection_.receive();
        const MsgpackParser parsed(recv_data.data(), recv_data.size());
        publish(parsed);
        // NOTE(@naefjo): Potential improvement:
        // https://github.com/bhuman/BHumanCodeRelease/blob/master/Src/Apps/Nao/Main.cpp#L199-L208

        // The shoulder pitch joints interpolate faster to avoid collisions of the arms with the legs.
        const float shoulder_pitch_phase = std::sqrt(std::min(1.F, ratio / 0.6F));
        for (int i = 0; i < static_cast<int>(LolaEnums::Joint::NUM_JOINTS); ++i) {
          if (i == 2 || i == 18) {
            packer_.position[i] = sit_down_angles[i] * shoulder_pitch_phase + start_angles[i] * (1.F - shoulder_pitch_phase);
          } else {
            packer_.position[i] = sit_down_angles[i] * phase + start_angles[i] * (1.F - phase);
          }
        }

        // Send packet to LoLA
        sendData();

        ratio += 0.012 / 2.F; // 2 seconds
      }
    }

    // Switch off stiffness of all joints
    for (int i = 0; i < static_cast<int>(LolaEnums::Joint::NUM_JOINTS); ++i) {
      packer_.stiffness[i] = 0.0;
    }

    // Switch off all leds, except for the eyes (ok: blue, crashed: red)
    std::fill(packer_.chest.begin(), packer_.chest.end(), 0.F);
    std::fill(packer_.l_ear.begin(), packer_.l_ear.end(), 0.F);
    std::fill(packer_.r_ear.begin(), packer_.r_ear.end(), 0.F);
    std::fill(packer_.l_eye.begin(), packer_.l_eye.end(), 0.F);
    std::fill(packer_.r_eye.begin(), packer_.r_eye.end(), 0.F);
    std::fill(packer_.l_foot.begin(), packer_.l_foot.end(), 0.F);
    std::fill(packer_.r_foot.begin(), packer_.r_foot.end(), 0.F);
    if (!chest_button_shutdown_requested_) {
      for (size_t i = 0; i < 8; ++i) {
        packer_.l_eye[i] = 0.1;
        packer_.r_eye[i] = 0.1;
      }
    } else {
      for (auto i = static_cast<std::size_t>(2U * static_cast<uint8_t>(nao_lola_command_msgs::msg::EyeLeds::NUM_LEDS));
           i < static_cast<std::size_t>(3U * static_cast<uint8_t>(nao_lola_command_msgs::msg::EyeLeds::NUM_LEDS));
           ++i) {
        packer_.l_eye[i] = 1.0;
        packer_.r_eye[i] = 1.0;
      }
    }

    sendData();
  }

  void NaoLolaClient::standUp(const MsgpackParser& parsed_sensor_data) {
    std::vector<float> start_angles = parsed_sensor_data.getSensorData("Position");
    std::array<float, static_cast<int>(LolaEnums::Joint::NUM_JOINTS)> stand_up_angles;
    std::transform(parameters_.stand_up_angles.begin(),
                   parameters_.stand_up_angles.end(),
                   stand_up_angles.begin(),
                   [](double x) { return static_cast<float>(x); });

    float ratio = 0.F;
    while (ratio < 1.F) {
      // Do sinus interpolation
      const float phase = 0.5F * std::sin((ratio - 0.5F) * nomadz_core::constants::PI) + 0.5F;

      auto recv_data = connection_.receive();
      const MsgpackParser parsed(recv_data.data(), recv_data.size());
      publish(parsed);
      // Inverse of sitdown, shoulders need to interpolate slower.
      const float shoulder_pitch_phase = phase * phase;
      for (int i = 0; i < static_cast<int>(LolaEnums::Joint::NUM_JOINTS); ++i) {
        if (i == 2 || i == 18) {
          packer_.position[i] = stand_up_angles[i] * shoulder_pitch_phase + start_angles[i] * (1.F - shoulder_pitch_phase);
          packer_.stiffness[i] = 0.7;
        } else {
          packer_.position[i] = stand_up_angles[i] * phase + start_angles[i] * (1.F - phase);
          packer_.stiffness[i] = 0.7;
        }
      }

      // Send packet to LoLA
      sendData();

      ratio += 0.012 / 2.F; // 2 seconds
    }
  }

  void NaoLolaClient::createPublishers() {
    RCLCPP_DEBUG(get_logger(), "Initialise publishers");
    unstiff_state_pub_ = create_publisher<std_msgs::msg::Bool>("sensors/unstiff_state", 10);
    imu_pub_ = create_publisher<nao_lola_sensor_msgs::msg::Imu>("sensors/imu", 10);
    buttons_pub_ = create_publisher<nao_lola_sensor_msgs::msg::Buttons>("sensors/buttons", 10);
    fsr_pub_ = create_publisher<nao_lola_sensor_msgs::msg::Fsr>("sensors/fsr", 10);
    joint_data_pub_ = create_publisher<nao_lola_sensor_msgs::msg::JointData>("sensors/joint_data", 10);
    sonar_pub_ = create_publisher<nao_lola_sensor_msgs::msg::Sonar>("sensors/sonar", 10);
    battery_pub_ = create_publisher<nao_lola_sensor_msgs::msg::Battery>("sensors/battery", 10);
    robot_config_pub_ = create_publisher<nao_lola_sensor_msgs::msg::RobotConfig>("sensors/robot_config", 10);
    RCLCPP_DEBUG(get_logger(), "Finished initialising publishers");
  }

  void NaoLolaClient::createSubscriptions() {
    RCLCPP_DEBUG(get_logger(), "Initialise subscriptions");
    joint_requests_sub_ = create_subscription<nao_lola_command_msgs::msg::JointRequests>(
      "effectors/joint_requests", 1, [this](const nao_lola_command_msgs::msg::JointRequests& jointRequests) {
        if (is_in_emergency_state_) {
          return;
        }
        const std::lock_guard<std::mutex> guard(packer_mutex_);
        packer_.setJointRequests(jointRequests);
      });

    chest_led_sub_ = create_subscription<nao_lola_command_msgs::msg::ChestLed>(
      "effectors/chest_led", 1, [this](const nao_lola_command_msgs::msg::ChestLed& chestLed) {
        const std::lock_guard<std::mutex> guard(packer_mutex_);
        packer_.setChestLed(chestLed);
      });

    ear_leds_sub_ = create_subscription<nao_lola_command_msgs::msg::EarLeds>(
      "effectors/ear_leds", 1, [this](const nao_lola_command_msgs::msg::EarLeds& EarLeds) {
        const std::lock_guard<std::mutex> guard(packer_mutex_);
        packer_.setLeftEarLeds(EarLeds);
        packer_.setRightEarLeds(EarLeds);
      });

    eye_leds_sub_ = create_subscription<nao_lola_command_msgs::msg::EyeLeds>(
      "effectors/eye_leds", 1, [this](const nao_lola_command_msgs::msg::EyeLeds& EyeLeds) {
        const std::lock_guard<std::mutex> guard(packer_mutex_);
        packer_.setLeftEyeLeds(EyeLeds);
        packer_.setRightEyeLeds(EyeLeds);
      });

    foot_led_sub_ = create_subscription<nao_lola_command_msgs::msg::FootLed>(
      "effectors/foot_led", 1, [this](const nao_lola_command_msgs::msg::FootLed& FootLed) {
        const std::lock_guard<std::mutex> guard(packer_mutex_);
        packer_.setLeftFootLed(FootLed);
        packer_.setRightFootLed(FootLed);
      });

    head_leds_sub_ = create_subscription<nao_lola_command_msgs::msg::HeadLeds>(
      "effectors/head_leds", 1, [this](const nao_lola_command_msgs::msg::HeadLeds& headLeds) {
        const std::lock_guard<std::mutex> guard(packer_mutex_);
        packer_.setHeadLeds(headLeds);
      });

    sonar_usage_sub_ = create_subscription<nao_lola_command_msgs::msg::SonarUsage>(
      "effectors/sonar_usage", 1, [this](const nao_lola_command_msgs::msg::SonarUsage& sonarUsage) {
        if (is_in_emergency_state_) {
          return;
        }
        const std::lock_guard<std::mutex> guard(packer_mutex_);
        packer_.setSonarUsage(sonarUsage);
      });
    RCLCPP_DEBUG(get_logger(), "Finished creating subscriptions");
  }

  void NaoLolaClient::publish(const MsgpackParser& parsed_sensor_data) {
    std_msgs::msg::Bool unstiff_state{};
    unstiff_state.data = unstiffed_;
    unstiff_state_pub_->publish(unstiff_state);
    imu_pub_->publish(parsed_sensor_data.getImu());
    buttons_pub_->publish(parsed_sensor_data.getButtons());
    fsr_pub_->publish(parsed_sensor_data.getFsr());
    joint_data_pub_->publish(parsed_sensor_data.getJointData());
    sonar_pub_->publish(parsed_sensor_data.getSonar());
    battery_pub_->publish(parsed_sensor_data.getBattery());
    robot_config_pub_->publish(parsed_sensor_data.getRobotConfig());
  }
} // namespace nao_lola_client
