// Copyright 2023 Kenji Brameld
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
#include "nao_lola_client/conversion.hpp"

#include <string>

#include "nao_lola_sensor_msgs/msg/joint_indexes.hpp"

namespace nao_lola_client::conversion {

  // clang-format off
  static const std::vector<std::string> JOINT_NAMES = {
    "HeadYaw",
    "HeadPitch",
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",
    "LWristYaw",
    "LHipYawPitch",
    "LHipRoll",
    "LHipPitch",
    "LKneePitch",
    "LAnklePitch",
    "LAnkleRoll",
    "RHipRoll",
    "RHipPitch",
    "RKneePitch",
    "RAnklePitch",
    "RAnkleRoll",
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll",
    "RWristYaw",
    "LHand",
    "RHand",
  };
  // clang-format on

  sensor_msgs::msg::JointState toJointState(const nao_lola_sensor_msgs::msg::JointData& joint_data) {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = joint_data.header.stamp;
    joint_state.name = JOINT_NAMES;

    for (unsigned i = 0; i < nao_lola_sensor_msgs::msg::JointIndexes::NUMJOINTS; ++i) {
      joint_state.position.push_back(joint_data.positions[i]);
    }

    return joint_state;
  }

} // namespace nao_lola_client::conversion
