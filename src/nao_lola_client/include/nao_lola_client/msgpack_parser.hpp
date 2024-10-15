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

#include <map>
#include <string>
#include <vector>

#include <msgpack.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nao_lola_sensor_msgs/msg/joint_data.hpp"
#include "nao_lola_sensor_msgs/msg/buttons.hpp"
#include "nao_lola_sensor_msgs/msg/imu.hpp"
#include "nao_lola_sensor_msgs/msg/sonar.hpp"
#include "nao_lola_sensor_msgs/msg/fsr.hpp"
#include "nao_lola_sensor_msgs/msg/battery.hpp"
#include "nao_lola_sensor_msgs/msg/robot_config.hpp"

namespace nao_lola_client {

  // Parser for msgpack of messages we receive from the Nao.
  class MsgpackParser {
  public:
    explicit MsgpackParser(char data[], int size, rclcpp::Time time_stamp = rclcpp::Time());

    /**
     * @brief Access sensor data of specific type
     */
    std::vector<float> getSensorData(const std::string& type) const;
    nao_lola_sensor_msgs::msg::Imu getImu() const;
    nao_lola_sensor_msgs::msg::Buttons getButtons() const;
    nao_lola_sensor_msgs::msg::Fsr getFsr() const;
    nao_lola_sensor_msgs::msg::JointData getJointData() const;
    nao_lola_sensor_msgs::msg::Sonar getSonar() const;
    nao_lola_sensor_msgs::msg::Battery getBattery() const;
    nao_lola_sensor_msgs::msg::RobotConfig getRobotConfig() const;

  private:
    msgpack::object_handle oh_; // Keep this variable throughout the lifetime of this object
    std::map<std::string, msgpack::object> unpacked_;
    rclcpp::Time time_stamp_;
  };
} // namespace nao_lola_client
