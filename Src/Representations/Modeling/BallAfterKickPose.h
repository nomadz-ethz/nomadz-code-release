/**
 * @file BallAfterKickPose.h
 *
 * Declaration of the BallAfterKickPose and PassTarget representations
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Math/Vector2.h"
#include "Core/System/BHAssert.h"
#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/ball_after_kick_pose.hpp"
#endif
STREAMABLE_DECLARE(BallAfterKickPose)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/pass_target.hpp"
#endif
STREAMABLE_DECLARE(PassTarget)

STREAMABLE_ROS(BallAfterKickPose, {
  public : void draw() const,
  FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::BallAfterKickPose::position, position),
  FIELD_WRAPPER(
    unsigned, 0, nomadz_msgs::msg::BallAfterKickPose::time_when_last_kick_was_performed, timeWhenLastKickWasPerformed),
  FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::BallAfterKickPose::dev, dev),
  FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::BallAfterKickPose::rot, rot),
  FIELD_WRAPPER(int, 0, nomadz_msgs::msg::BallAfterKickPose::player_number, playerNumber),
});

STREAMABLE_ROS(PassTarget, {
  public : PassTarget(const Vector2<>& target, const int& playerNumber, unsigned timeToReachPassPose),
  FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::PassTarget::target, target),
  FIELD_WRAPPER(int, 0, nomadz_msgs::msg::PassTarget::player_number, playerNumber),
  FIELD_WRAPPER(unsigned, 0, nomadz_msgs::msg::PassTarget::time_to_reach_pass_pose, timeToReachPassPose),
});
