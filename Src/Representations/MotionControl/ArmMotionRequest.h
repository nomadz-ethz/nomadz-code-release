/**
 * @file ArmMotionRequest.h
 *
 * Request for the arm motion engine.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:simont@tzi.de>Simon Taddiken</a>
 */

#pragma once
#include "Core/Streams/AutoStreamable.h"
#include "Core/Enum.h"

/**
 * Class that represents the possible arm motions that can be requested from
 * the robot.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/arm_motion_request.hpp"
#endif
STREAMABLE_DECLARE(ArmMotionRequest)

STREAMABLE_ROS(ArmMotionRequest, {
public:
  ENUM(Arm, left, right);

  /** Existing arm motions. Ordering must match ordering in armMotionEngine.cfg */
  ENUM(ArmMotionId,
       useDefault, /**< No explicit arm motion, so WalkingEngine's arm angles will be used */
       back,       /**< Move arm to the back */
       falling,    /**< Emergency motion to save arm when falling */
       keeperStand /**< Arm position for the keeper when guarding the goal */
  )
  ,
    FIELD_WRAPPER_DEFAULT(
      ArmMotionId[numOfArms], nomadz_msgs::msg::ArmMotionRequest::motion, motion), /**< Motion to execute */
    FIELD_WRAPPER_DEFAULT(
      bool[numOfArms], nomadz_msgs::msg::ArmMotionRequest::fast, fast), /**< Whether states should not be interpolated */
    FIELD_WRAPPER_DEFAULT(
      bool[numOfArms],
      nomadz_msgs::msg::ArmMotionRequest::auto_reverse,
      autoReverse), /**< Whether arms should be moved back to default position after certain amount of time */
    FIELD_WRAPPER_DEFAULT(
      int[numOfArms], nomadz_msgs::msg::ArmMotionRequest::auto_reverse_time, autoReverseTime), /**< Time (in motion frames)
                                after which the arm is moved back to default position if autoReverse is true. */

    motion[left] = useDefault;
  motion[right] = useDefault;
  fast[left] = false;
  fast[right] = false;
  autoReverse[left] = false;
  autoReverse[right] = false;
  autoReverseTime[left] = 0;
  autoReverseTime[right] = 0;
});
