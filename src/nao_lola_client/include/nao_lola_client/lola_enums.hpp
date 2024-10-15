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

// NOLINTNEXTLINE(readability-identifier-naming)
namespace nao_lola_client::LolaEnums {

  // The enums are exactly as defined (and in the same order) as what's provided
  // in "Lola RoboCupper Official Documentation.pdf" sent to all Standard Platform League
  // teams in RoboCup.

  // NOLINTBEGIN(readability-identifier-naming)
  enum class Joint : std::uint8_t {
    HeadYaw,
    HeadPitch,
    LShoulderPitch,
    LShoulderRoll,
    LElbowYaw,
    LElbowRoll,
    LWristYaw,
    LHipYawPitch,
    LHipRoll,
    LHipPitch,
    LKneePitch,
    LAnklePitch,
    LAnkleRoll,
    RHipRoll,
    RHipPitch,
    RKneePitch,
    RAnklePitch,
    RAnkleRoll,
    RShoulderPitch,
    RShoulderRoll,
    RElbowYaw,
    RElbowRoll,
    RWristYaw,
    LHand,
    RHand,
    NUM_JOINTS
  };

  enum class Battery : std::uint8_t { Charge, Current, Status, Temperature };

  enum class Accelerometer : std::uint8_t { X, Y, Z };

  enum class Gyroscope : std::uint8_t { X, Y, Z };

  enum class Angles : std::uint8_t { X, Y };

  enum class Sonar : std::uint8_t { Left, Right };

  enum class FSR : std::uint8_t {
    LFoot_FrontLeft,
    LFoot_FrontRight,
    LFoot_RearLeft,
    LFoot_RearRight,
    RFoot_FrontLeft,
    RFoot_FrontRight,
    RFoot_RearLeft,
    RFoot_RearRight
  };

  enum class Touch : std::uint8_t {
    ChestBoard_Button,
    Head_Touch_Front,
    Head_Touch_Middle,
    Head_Touch_Rear,
    LFoot_Bumper_Left,
    LFoot_Bumper_Right,
    LHand_Touch_Back,
    LHand_Touch_Left,
    LHand_Touch_Right,
    RFoot_Bumper_Left,
    RFoot_Bumper_Right,
    RHand_Touch_Back,
    RHand_Touch_Left,
    RHand_Touch_Right
  };

  enum class RobotConfig : std::uint8_t { Body_BodyId, Body_Version, Head_FullHeadId, Head_Version };

  enum class LeftEarLeds : std::uint8_t {
    Deg_0,
    Deg_36,
    Deg_72,
    Deg_108,
    Deg_144,
    Deg_180,
    Deg_216,
    Deg_252,
    Deg_288,
    Deg_324
  };

  enum class RightEarLeds : std::uint8_t {
    Deg_324,
    Deg_288,
    Deg_252,
    Deg_216,
    Deg_180,
    Deg_144,
    Deg_108,
    Deg_72,
    Deg_36,
    Deg_0
  };

  enum class LeftEyeLeds : std::uint8_t { Deg_45, Deg_0, Deg_315, Deg_270, Deg_225, Deg_180, Deg_135, Deg_90 };

  enum class RightEyeLeds : std::uint8_t { Deg_0, Deg_45, Deg_90, Deg_135, Deg_180, Deg_225, Deg_270, Deg_315 };

  enum class SkullLeds : std::uint8_t {
    Front_Left_1,
    Front_Left_0,
    Middle_Left_0,
    Rear_Left_0,
    Rear_Left_1,
    Rear_Left_2,
    Rear_Right_2,
    Rear_Right_1,
    Rear_Right_0,
    Middle_Right_0,
    Front_Right_0,
    Front_Right_1
  };
  // NOLINTEND(readability-identifier-naming)
} // namespace nao_lola_client::LolaEnums
