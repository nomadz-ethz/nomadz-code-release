/**
 * @file ArmMotionEngineOutput.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author are <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a> and <a
 * href="mailto:simont@tzi.de">Simon Taddiken</a>
 */

#pragma once

#include <vector>
#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/ArmMotionRequest.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/arm.hpp"
#endif
STREAMABLE_DECLARE(Arm)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/arm_motion_engine_output.hpp"
#endif
STREAMABLE_DECLARE(ArmMotionEngineOutput)
STREAMABLE_ROS(ArmMotionEngineOutput, {
  public : STREAMABLE_ROS(
    Arm,
    {
    public:
      // FIXME: Copy from ArmMotionRequest
      /** Existing arm motions. Ordering must match ordering in armMotionEngine.cfg */
      ENUM_ALIAS(ArmMotionRequest,
                 ArmMotionId,
                 useDefault, /**< No explicit arm motion, so WalkingEngine's arm angles will be used */
                 back,       /**< Move arm to the back */
                 falling,    /**< Emergency motion to save arm when falling */
                 keeperStand /**< Arm position for the keeper when guarding the goal */
                 ),
        FIELD_WRAPPER(bool, false, nomadz_msgs::msg::Arm::move, move), /**< determines whether an arm should be moved */
        FIELD_WRAPPER(unsigned,
                      0,
                      nomadz_msgs::msg::Arm::last_movement,
                      lastMovement), /**< Timestamp of when each arm has been moved last */
        FIELD_WRAPPER(ArmMotionRequest::ArmMotionId,
                      ArmMotionRequest::useDefault,
                      nomadz_msgs::msg::Arm::motion,
                      motion), /**< The arm motion being executed. */
                               // (std::vector<float>)(6, 0)
        //   angles, /**< contains the target arm joints for each arm. Those values are ignored if moveArm is false */
        FIELD_WRAPPER_DEFAULT(
          float[6],
          nomadz_msgs::msg::Arm::angles,
          angles), /**< contains the target arm joints for each arm. Those values are ignored if moveArm is false */
        FIELD_WRAPPER_DEFAULT(
          int[6],
          nomadz_msgs::msg::Arm::hardness,
          hardness), /**< contains hardness data for this arm motion. Those values are ignored if moveArm is false*/
                     // (std::vector<int>)(6, HardnessData::useDefault)
        //   hardness, /**< contains hardness data for this arm motion. Those values are ignored if moveArm is false*/

        for (int i = 0; i < 6; ++i) hardness[i] = HardnessData::useDefault;
    }),
  FIELD_WRAPPER_DEFAULT(Arm[2], nomadz_msgs::msg::ArmMotionEngineOutput::arms, arms),
});
