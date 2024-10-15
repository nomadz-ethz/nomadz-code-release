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

#include "nao_lola_client/msgpack_packer.hpp"

#include <map>
#include <memory>
#include <utility>
#include <vector>
#include <string>

#include <msgpack.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nao_lola_client/command_index_conversion.hpp"

namespace nao_lola_client {

  std::string MsgpackPacker::getPacked() const {
    msgpack::zone z;
    std::map<std::string, msgpack::object> map;

    map.insert(std::make_pair("Position", msgpack::object(position, z)));
    map.insert(std::make_pair("Stiffness", msgpack::object(stiffness, z)));
    map.insert(std::make_pair("Chest", msgpack::object(chest, z)));
    map.insert(std::make_pair("LEar", msgpack::object(l_ear, z)));
    map.insert(std::make_pair("REar", msgpack::object(r_ear, z)));
    map.insert(std::make_pair("LEye", msgpack::object(l_eye, z)));
    map.insert(std::make_pair("REye", msgpack::object(r_eye, z)));
    map.insert(std::make_pair("LFoot", msgpack::object(l_foot, z)));
    map.insert(std::make_pair("RFoot", msgpack::object(r_foot, z)));
    map.insert(std::make_pair("Skull", msgpack::object(skull, z)));
    map.insert(std::make_pair("Sonar", msgpack::object(sonar, z)));

    std::stringstream buffer;
    msgpack::pack(buffer, map);
    std::string packed = buffer.str();

    return packed;
  }

  void MsgpackPacker::setJointRequests(const nao_lola_command_msgs::msg::JointRequests& jointRequests) {
    if (jointRequests.indexes.size() != jointRequests.positions.size() &&
        jointRequests.indexes.size() != jointRequests.stiffnesses.size()) {
      RCLCPP_ERROR(logger,
                   "Incorrect message received for nao_lola_command_msgs::msg::JointRequests. "
                   "Angles, Stiffnesses and Indexes vectors must have the same length. "
                   "Angles vector has length %zu, Stiffnesses vector has length %zu, while indexes vector has length %zu",
                   jointRequests.positions.size(),
                   jointRequests.stiffnesses.size(),
                   jointRequests.indexes.size());
    }

    for (unsigned i = 0; i < jointRequests.indexes.size(); ++i) {
      const int msg_joint_index = jointRequests.indexes[i];
      const float joint_angle = jointRequests.positions[i];
      const float joint_stiffness = jointRequests.stiffnesses[i];
      const LolaEnums::Joint lola_joint_index = IndexConversion::JOINT_MSG_TO_LOLA.at(msg_joint_index);
      position.at(static_cast<int>(lola_joint_index)) = joint_angle;
      stiffness.at(static_cast<int>(lola_joint_index)) = joint_stiffness;
    }
  }

  void MsgpackPacker::setChestLed(const nao_lola_command_msgs::msg::ChestLed& chestLed) {
    chest.at(0) = chestLed.color.r;
    chest.at(1) = chestLed.color.g;
    chest.at(2) = chestLed.color.b;
  }

  void MsgpackPacker::setLeftEarLeds(const nao_lola_command_msgs::msg::EarLeds& EarLeds) {
    for (int i = 0; i < nao_lola_command_msgs::msg::EarLeds::NUM_LEDS; ++i) {
      const LolaEnums::LeftEarLeds lola_index = IndexConversion::LEFT_EAR_LEDS_MSG_TO_LOLA.at(i);
      l_ear.at(static_cast<int>(lola_index)) = EarLeds.left_intensities[i];
    }
  }

  void MsgpackPacker::setRightEarLeds(const nao_lola_command_msgs::msg::EarLeds& EarLeds) {
    for (int i = 0; i < nao_lola_command_msgs::msg::EarLeds::NUM_LEDS; ++i) {
      const LolaEnums::RightEarLeds lola_index = IndexConversion::RIGHT_EAR_LEDS_MSG_TO_LOLA.at(i);
      r_ear.at(static_cast<int>(lola_index)) = EarLeds.right_intensities[i];
    }
  }

  void MsgpackPacker::setLeftEyeLeds(const nao_lola_command_msgs::msg::EyeLeds& EyeLeds) {
    for (int i = 0; i < nao_lola_command_msgs::msg::EyeLeds::NUM_LEDS; ++i) {
      const LolaEnums::LeftEyeLeds lola_index = IndexConversion::LEFT_EYE_LEDS_MSG_TO_LOLA.at(i);
      l_eye.at(static_cast<int>(lola_index)) = EyeLeds.left_colors[i].r;
      l_eye.at(static_cast<int>(lola_index) + 8) = EyeLeds.left_colors[i].g;
      l_eye.at(static_cast<int>(lola_index) + 16) = EyeLeds.left_colors[i].b;
    }
  }

  void MsgpackPacker::setRightEyeLeds(const nao_lola_command_msgs::msg::EyeLeds& EyeLeds) {
    for (int i = 0; i < nao_lola_command_msgs::msg::EyeLeds::NUM_LEDS; ++i) {
      const LolaEnums::RightEyeLeds lola_index = IndexConversion::RIGHT_EYE_LEDS_MSG_TO_LOLA.at(i);
      r_eye.at(static_cast<int>(lola_index)) = EyeLeds.right_colors[i].r;
      r_eye.at(static_cast<int>(lola_index) + 8) = EyeLeds.right_colors[i].g;
      r_eye.at(static_cast<int>(lola_index) + 16) = EyeLeds.right_colors[i].b;
    }
  }

  void MsgpackPacker::setLeftFootLed(const nao_lola_command_msgs::msg::FootLed& FootLed) {
    l_foot.at(0) = FootLed.left_color.r;
    l_foot.at(1) = FootLed.left_color.g;
    l_foot.at(2) = FootLed.left_color.b;
  }

  void MsgpackPacker::setRightFootLed(const nao_lola_command_msgs::msg::FootLed& FootLed) {
    r_foot.at(0) = FootLed.right_color.r;
    r_foot.at(1) = FootLed.right_color.g;
    r_foot.at(2) = FootLed.right_color.b;
  }

  void MsgpackPacker::setHeadLeds(const nao_lola_command_msgs::msg::HeadLeds& headLeds) {
    for (int i = 0; i < nao_lola_command_msgs::msg::HeadLeds::NUM_LEDS; ++i) {
      LolaEnums::SkullLeds const lola_index = IndexConversion::HEAD_LEDS_MSG_TO_LOLA.at(i);
      skull.at(static_cast<int>(lola_index)) = headLeds.intensities[i];
    }
  }

  void MsgpackPacker::setSonarUsage(const nao_lola_command_msgs::msg::SonarUsage& sonarUsage) {
    sonar.at(0) = sonarUsage.left;
    sonar.at(1) = sonarUsage.right;
  }
} // namespace nao_lola_client
