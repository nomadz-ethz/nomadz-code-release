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

#include "nao_lola_client/lola_enums.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "nao_lola_command_msgs/msg/ear_leds.hpp"
#include "nao_lola_command_msgs/msg/eye_leds.hpp"
#include "nao_lola_command_msgs/msg/head_leds.hpp"

// NOLINTNEXTLINE(readability-identifier-naming)
namespace nao_lola_client::IndexConversion {
  std::map<uint8_t, LolaEnums::Joint> flip(const std::map<LolaEnums::Joint, uint8_t>& in);

  static const std::map<LolaEnums::Joint, uint8_t> JOINT_LOLA_TO_MSG = {
    {LolaEnums::Joint::HeadYaw, nao_lola_command_msgs::msg::JointIndexes::HEADYAW},
    {LolaEnums::Joint::HeadPitch, nao_lola_command_msgs::msg::JointIndexes::HEADPITCH},
    {LolaEnums::Joint::LShoulderPitch, nao_lola_command_msgs::msg::JointIndexes::LSHOULDERPITCH},
    {LolaEnums::Joint::LShoulderRoll, nao_lola_command_msgs::msg::JointIndexes::LSHOULDERROLL},
    {LolaEnums::Joint::LElbowYaw, nao_lola_command_msgs::msg::JointIndexes::LELBOWYAW},
    {LolaEnums::Joint::LElbowRoll, nao_lola_command_msgs::msg::JointIndexes::LELBOWROLL},
    {LolaEnums::Joint::LWristYaw, nao_lola_command_msgs::msg::JointIndexes::LWRISTYAW},
    {LolaEnums::Joint::LHipYawPitch, nao_lola_command_msgs::msg::JointIndexes::LHIPYAWPITCH},
    {LolaEnums::Joint::LHipRoll, nao_lola_command_msgs::msg::JointIndexes::LHIPROLL},
    {LolaEnums::Joint::LHipPitch, nao_lola_command_msgs::msg::JointIndexes::LHIPPITCH},
    {LolaEnums::Joint::LKneePitch, nao_lola_command_msgs::msg::JointIndexes::LKNEEPITCH},
    {LolaEnums::Joint::LAnklePitch, nao_lola_command_msgs::msg::JointIndexes::LANKLEPITCH},
    {LolaEnums::Joint::LAnkleRoll, nao_lola_command_msgs::msg::JointIndexes::LANKLEROLL},
    {LolaEnums::Joint::RHipRoll, nao_lola_command_msgs::msg::JointIndexes::RHIPROLL},
    {LolaEnums::Joint::RHipPitch, nao_lola_command_msgs::msg::JointIndexes::RHIPPITCH},
    {LolaEnums::Joint::RKneePitch, nao_lola_command_msgs::msg::JointIndexes::RKNEEPITCH},
    {LolaEnums::Joint::RAnklePitch, nao_lola_command_msgs::msg::JointIndexes::RANKLEPITCH},
    {LolaEnums::Joint::RAnkleRoll, nao_lola_command_msgs::msg::JointIndexes::RANKLEROLL},
    {LolaEnums::Joint::RShoulderPitch, nao_lola_command_msgs::msg::JointIndexes::RSHOULDERPITCH},
    {LolaEnums::Joint::RShoulderRoll, nao_lola_command_msgs::msg::JointIndexes::RSHOULDERROLL},
    {LolaEnums::Joint::RElbowYaw, nao_lola_command_msgs::msg::JointIndexes::RELBOWYAW},
    {LolaEnums::Joint::RElbowRoll, nao_lola_command_msgs::msg::JointIndexes::RELBOWROLL},
    {LolaEnums::Joint::RWristYaw, nao_lola_command_msgs::msg::JointIndexes::RWRISTYAW},
    {LolaEnums::Joint::LHand, nao_lola_command_msgs::msg::JointIndexes::LHAND},
    {LolaEnums::Joint::RHand, nao_lola_command_msgs::msg::JointIndexes::RHAND},
  };

  static const std::map<uint8_t, LolaEnums::Joint> JOINT_MSG_TO_LOLA = flip(JOINT_LOLA_TO_MSG);

  inline std::map<uint8_t, LolaEnums::Joint> flip(const std::map<LolaEnums::Joint, uint8_t>& in) {
    std::map<uint8_t, LolaEnums::Joint> flipped;
    for (const auto& i : in) {
      flipped[i.second] = i.first;
    }

    return flipped;
  }

  // See http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#left-ear
  static const std::map<int32_t, LolaEnums::LeftEarLeds> LEFT_EAR_LEDS_MSG_TO_LOLA{
    {nao_lola_command_msgs::msg::EarLeds::L0, LolaEnums::LeftEarLeds::Deg_0},
    {nao_lola_command_msgs::msg::EarLeds::L1, LolaEnums::LeftEarLeds::Deg_36},
    {nao_lola_command_msgs::msg::EarLeds::L2, LolaEnums::LeftEarLeds::Deg_72},
    {nao_lola_command_msgs::msg::EarLeds::L3, LolaEnums::LeftEarLeds::Deg_108},
    {nao_lola_command_msgs::msg::EarLeds::L4, LolaEnums::LeftEarLeds::Deg_144},
    {nao_lola_command_msgs::msg::EarLeds::L5, LolaEnums::LeftEarLeds::Deg_180},
    {nao_lola_command_msgs::msg::EarLeds::L6, LolaEnums::LeftEarLeds::Deg_216},
    {nao_lola_command_msgs::msg::EarLeds::L7, LolaEnums::LeftEarLeds::Deg_252},
    {nao_lola_command_msgs::msg::EarLeds::L8, LolaEnums::LeftEarLeds::Deg_288},
    {nao_lola_command_msgs::msg::EarLeds::L9, LolaEnums::LeftEarLeds::Deg_324}};

  // See http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#right-ear
  static const std::map<int, LolaEnums::RightEarLeds> RIGHT_EAR_LEDS_MSG_TO_LOLA{
    {nao_lola_command_msgs::msg::EarLeds::R0, LolaEnums::RightEarLeds::Deg_0},
    {nao_lola_command_msgs::msg::EarLeds::R1, LolaEnums::RightEarLeds::Deg_36},
    {nao_lola_command_msgs::msg::EarLeds::R2, LolaEnums::RightEarLeds::Deg_72},
    {nao_lola_command_msgs::msg::EarLeds::R3, LolaEnums::RightEarLeds::Deg_108},
    {nao_lola_command_msgs::msg::EarLeds::R4, LolaEnums::RightEarLeds::Deg_144},
    {nao_lola_command_msgs::msg::EarLeds::R5, LolaEnums::RightEarLeds::Deg_180},
    {nao_lola_command_msgs::msg::EarLeds::R6, LolaEnums::RightEarLeds::Deg_216},
    {nao_lola_command_msgs::msg::EarLeds::R7, LolaEnums::RightEarLeds::Deg_252},
    {nao_lola_command_msgs::msg::EarLeds::R8, LolaEnums::RightEarLeds::Deg_288},
    {nao_lola_command_msgs::msg::EarLeds::R9, LolaEnums::RightEarLeds::Deg_324}};

  // See http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#nao-v5-v4-and-v3-3
  static const std::map<int, LolaEnums::LeftEyeLeds> LEFT_EYE_LEDS_MSG_TO_LOLA{
    {nao_lola_command_msgs::msg::EyeLeds::L0, LolaEnums::LeftEyeLeds::Deg_45},
    {nao_lola_command_msgs::msg::EyeLeds::L1, LolaEnums::LeftEyeLeds::Deg_0},
    {nao_lola_command_msgs::msg::EyeLeds::L2, LolaEnums::LeftEyeLeds::Deg_315},
    {nao_lola_command_msgs::msg::EyeLeds::L3, LolaEnums::LeftEyeLeds::Deg_270},
    {nao_lola_command_msgs::msg::EyeLeds::L4, LolaEnums::LeftEyeLeds::Deg_225},
    {nao_lola_command_msgs::msg::EyeLeds::L5, LolaEnums::LeftEyeLeds::Deg_180},
    {nao_lola_command_msgs::msg::EyeLeds::L6, LolaEnums::LeftEyeLeds::Deg_135},
    {nao_lola_command_msgs::msg::EyeLeds::L7, LolaEnums::LeftEyeLeds::Deg_90},
  };

  // See http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#nao-v5-v4-and-v3-3
  static const std::map<int, LolaEnums::RightEyeLeds> RIGHT_EYE_LEDS_MSG_TO_LOLA{
    {nao_lola_command_msgs::msg::EyeLeds::R0, LolaEnums::RightEyeLeds::Deg_315},
    {nao_lola_command_msgs::msg::EyeLeds::R1, LolaEnums::RightEyeLeds::Deg_270},
    {nao_lola_command_msgs::msg::EyeLeds::R2, LolaEnums::RightEyeLeds::Deg_225},
    {nao_lola_command_msgs::msg::EyeLeds::R3, LolaEnums::RightEyeLeds::Deg_180},
    {nao_lola_command_msgs::msg::EyeLeds::R4, LolaEnums::RightEyeLeds::Deg_135},
    {nao_lola_command_msgs::msg::EyeLeds::R5, LolaEnums::RightEyeLeds::Deg_90},
    {nao_lola_command_msgs::msg::EyeLeds::R6, LolaEnums::RightEyeLeds::Deg_45},
    {nao_lola_command_msgs::msg::EyeLeds::R7, LolaEnums::RightEyeLeds::Deg_0},
  };

  // See http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#head-tactile-sensor-led-locations
  static const std::map<int, LolaEnums::SkullLeds> HEAD_LEDS_MSG_TO_LOLA{
    {nao_lola_command_msgs::msg::HeadLeds::B0, LolaEnums::SkullLeds::Front_Right_1},
    {nao_lola_command_msgs::msg::HeadLeds::B1, LolaEnums::SkullLeds::Front_Right_0},
    {nao_lola_command_msgs::msg::HeadLeds::B2, LolaEnums::SkullLeds::Middle_Right_0},
    {nao_lola_command_msgs::msg::HeadLeds::B3, LolaEnums::SkullLeds::Rear_Right_0},
    {nao_lola_command_msgs::msg::HeadLeds::B4, LolaEnums::SkullLeds::Rear_Right_1},
    {nao_lola_command_msgs::msg::HeadLeds::B5, LolaEnums::SkullLeds::Rear_Right_2},
    {nao_lola_command_msgs::msg::HeadLeds::B6, LolaEnums::SkullLeds::Rear_Left_2},
    {nao_lola_command_msgs::msg::HeadLeds::B7, LolaEnums::SkullLeds::Rear_Left_1},
    {nao_lola_command_msgs::msg::HeadLeds::B8, LolaEnums::SkullLeds::Rear_Left_0},
    {nao_lola_command_msgs::msg::HeadLeds::B9, LolaEnums::SkullLeds::Middle_Left_0},
    {nao_lola_command_msgs::msg::HeadLeds::B10, LolaEnums::SkullLeds::Front_Left_0},
    {nao_lola_command_msgs::msg::HeadLeds::B11, LolaEnums::SkullLeds::Front_Left_1},
  };
} // namespace nao_lola_client::IndexConversion
