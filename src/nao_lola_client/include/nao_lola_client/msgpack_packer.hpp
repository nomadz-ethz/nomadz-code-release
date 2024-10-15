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

#include <cstddef>
#include <string>
#include <vector>
#include <memory>

#include <rclcpp/logger.hpp>

#include "nao_lola_command_msgs/msg/joint_requests.hpp"
#include "nao_lola_command_msgs/msg/chest_led.hpp"
#include "nao_lola_command_msgs/msg/ear_leds.hpp"
#include "nao_lola_command_msgs/msg/eye_leds.hpp"
#include "nao_lola_command_msgs/msg/foot_led.hpp"
#include "nao_lola_command_msgs/msg/head_leds.hpp"
#include "nao_lola_command_msgs/msg/sonar_usage.hpp"
#include "nao_lola_client/lola_enums.hpp"

namespace nao_lola_client {

  // Packer for msgpack of messages we receive from the Nao.
  // Exposes functions which allow us to set the values from ROS 2 Messages
  // or we can directly write into the data arrays.
  class MsgpackPacker {
  public:
    MsgpackPacker() : logger(rclcpp::get_logger("msgpack packer")) {}
    std::string getPacked() const;

    void setJointRequests(const nao_lola_command_msgs::msg::JointRequests& jointRequests);
    void setChestLed(const nao_lola_command_msgs::msg::ChestLed& chestLed);
    void setLeftEarLeds(const nao_lola_command_msgs::msg::EarLeds& EarLeds);
    void setRightEarLeds(const nao_lola_command_msgs::msg::EarLeds& EarLeds);
    void setLeftEyeLeds(const nao_lola_command_msgs::msg::EyeLeds& EyeLeds);
    void setRightEyeLeds(const nao_lola_command_msgs::msg::EyeLeds& EyeLeds);
    void setLeftFootLed(const nao_lola_command_msgs::msg::FootLed& FootLed);
    void setRightFootLed(const nao_lola_command_msgs::msg::FootLed& FootLed);
    void setHeadLeds(const nao_lola_command_msgs::msg::HeadLeds& headLeds);
    void setSonarUsage(const nao_lola_command_msgs::msg::SonarUsage& sonarUsage);

    // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
    std::array<float, static_cast<int>(LolaEnums::Joint::NUM_JOINTS)> position;
    std::array<float, static_cast<int>(LolaEnums::Joint::NUM_JOINTS)> stiffness;
    std::array<float, 3> chest;
    std::array<float, static_cast<int>(nao_lola_command_msgs::msg::EarLeds::NUM_LEDS)> l_ear;
    std::array<float, static_cast<int>(nao_lola_command_msgs::msg::EarLeds::NUM_LEDS)> r_ear;
    std::array<float, static_cast<std::size_t>(3 * static_cast<int>(nao_lola_command_msgs::msg::EyeLeds::NUM_LEDS))> l_eye;
    std::array<float, static_cast<std::size_t>(3 * static_cast<int>(nao_lola_command_msgs::msg::EyeLeds::NUM_LEDS))> r_eye;
    std::array<float, 3> l_foot;
    std::array<float, 3> r_foot;
    std::array<float, static_cast<int>(nao_lola_command_msgs::msg::HeadLeds::NUM_LEDS)> skull;
    std::array<bool, 2> sonar;

    rclcpp::Logger logger;
    // NOLINTEND(misc-non-private-member-variables-in-classes)
  };
} // namespace nao_lola_client
