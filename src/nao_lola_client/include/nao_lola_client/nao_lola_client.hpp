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

#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/bool.hpp"
#include "nao_lola_sensor_msgs/msg/joint_data.hpp"
#include "nao_lola_sensor_msgs/msg/buttons.hpp"
#include "nao_lola_sensor_msgs/msg/imu.hpp"
#include "nao_lola_sensor_msgs/msg/sonar.hpp"
#include "nao_lola_sensor_msgs/msg/fsr.hpp"
#include "nao_lola_sensor_msgs/msg/battery.hpp"
#include "nao_lola_sensor_msgs/msg/robot_config.hpp"
#include "nao_lola_command_msgs/msg/chest_led.hpp"
#include "nao_lola_command_msgs/msg/ear_leds.hpp"
#include "nao_lola_command_msgs/msg/eye_leds.hpp"
#include "nao_lola_command_msgs/msg/foot_led.hpp"
#include "nao_lola_command_msgs/msg/head_leds.hpp"
#include "nao_lola_command_msgs/msg/sonar_usage.hpp"
#include "nao_lola_command_msgs/msg/joint_requests.hpp"
#include "nao_lola_client/connection.hpp"
#include "nao_lola_client/msgpack_packer.hpp"
#include "nao_lola_client/msgpack_parser.hpp"
#include "nao_lola_client/lola_enums.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nao_lola_client_parameters.hpp"

namespace nao_lola_client {

  class RosShutDownRequestException : public std::exception {
  public:
    const char* what() const noexcept override { return "Requesting to shut down ros node."; }
  };

  class NaoLolaClient : public rclcpp::Node {
  public:
    NaoLolaClient();
    ~NaoLolaClient() override;

  private:
    void createPublishers();
    void createSubscriptions();

    void publish(const MsgpackParser& parsed_sensor_data);
    /**
     * @brief Writes the data which is acuumulated in the packer to the robot.
     */
    void sendData();

    /**
     * @brief: Checks whether shutdown is requested.
     */
    void checkButtonStates(const MsgpackParser& parsed_sensor_data);

    /**
     * @brief: Executes the sit down motion in a blocking fashion.
     */
    void sitDown(const MsgpackParser& parsed_sensor_data);

    /**
     * @brief: Executes the stand up motion in a blocking fashion.
     */
    void standUp(const MsgpackParser& parsed_sensor_data);

    rclcpp::Publisher<nao_lola_sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nao_lola_sensor_msgs::msg::Buttons>::SharedPtr buttons_pub_;
    rclcpp::Publisher<nao_lola_sensor_msgs::msg::Fsr>::SharedPtr fsr_pub_;
    rclcpp::Publisher<nao_lola_sensor_msgs::msg::JointData>::SharedPtr joint_data_pub_;
    rclcpp::Publisher<nao_lola_sensor_msgs::msg::Sonar>::SharedPtr sonar_pub_;
    rclcpp::Publisher<nao_lola_sensor_msgs::msg::Battery>::SharedPtr battery_pub_;
    rclcpp::Publisher<nao_lola_sensor_msgs::msg::RobotConfig>::SharedPtr robot_config_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr unstiff_state_pub_;

    rclcpp::Subscription<nao_lola_command_msgs::msg::JointRequests>::SharedPtr joint_requests_sub_;
    rclcpp::Subscription<nao_lola_command_msgs::msg::ChestLed>::SharedPtr chest_led_sub_;
    rclcpp::Subscription<nao_lola_command_msgs::msg::EarLeds>::SharedPtr ear_leds_sub_;
    rclcpp::Subscription<nao_lola_command_msgs::msg::EyeLeds>::SharedPtr eye_leds_sub_;
    rclcpp::Subscription<nao_lola_command_msgs::msg::FootLed>::SharedPtr foot_led_sub_;
    rclcpp::Subscription<nao_lola_command_msgs::msg::HeadLeds>::SharedPtr head_leds_sub_;
    rclcpp::Subscription<nao_lola_command_msgs::msg::SonarUsage>::SharedPtr sonar_usage_sub_;

    // Struct containing the ROS 2 parameters. Note that these are only loaded during startup of the node.
    nao_lola_client::Params parameters_;

    std::thread receive_thread_;
    Connection connection_;

    MsgpackPacker packer_;
    std::mutex packer_mutex_;

    // Member variables for chest button shutdown logic.
    bool chest_button_shutdown_requested_{false};
    rclcpp::Time time_when_chest_button_was_last_unpressed_;

    // Member variables for head button emergency stops.
    rclcpp::Time time_when_all_head_buttons_was_last_unpressed_;
    rclcpp::Time time_when_last_emergency_state_switch_;
#ifdef TARGET_ROBOT
    bool is_in_emergency_state_{true};
#else
    bool is_in_emergency_state_{false};
#endif
    bool is_seated_{true};
    bool unstiffed_{false};
  };
} // namespace nao_lola_client
