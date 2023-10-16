/**
 * @file WalkLearner.h
 *
 * This file is subject to the terms of the BHuman 2019 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Philip Reichenberg
 */

#include "Core/Function.h"
#include "Representations/MotionControl/Walk2014Modifier.h"
#pragma once
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/walk_learner.hpp"
#endif
STREAMABLE_DECLARE(WalkLearner)

STREAMABLE_ROS(WalkLearner, {
  public : FUNCTION(void(float gyroForward, float gyroBackward, float speedTransX)) setBaseWalkParams,
  FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::WalkLearner::new_gyro_forward_balance, newGyroForwardBalance),
  FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::WalkLearner::new_gyro_backward_balance, newGyroBackwardBalance),
  FIELD_WRAPPER(bool, false, nomadz_msgs::msg::WalkLearner::use_walk_learner, useWalkLearner),
});
