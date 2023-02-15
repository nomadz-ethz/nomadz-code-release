/**
 * @file PlannedSteps.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Function.h"
#include "Core/Math/Pose2D.h"
#include "Core/RingBuffer.h"
#include "MotionRequest.h"
#include <deque>
#include "Core/Streams/FieldWrapper.h"
#include "Representations/MotionControl/WalkGeneratorData.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/walk_generator.hpp"
#endif

STREAMABLE_DECLARE(UpcomingStep)
STREAMABLE_ROS(UpcomingStep, {
public:
  ENUM(LegSide, left, right);
  , FIELD_WRAPPER(LegSide, left, nomadz_msgs::msg::UpcomingStep::leg_side, legSide),
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::UpcomingStep::leg_pose, legPose),
    FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::UpcomingStep::stepDutation, stepDuration),
});

STREAMABLE_DECLARE(PlannedSteps)

STREAMABLE_ROS(PlannedSteps, {
public:
  FUNCTION(void()) reset;
  FUNCTION(void(const Pose2D& speed, const Pose2D& target, WalkGeneratorData::WalkMode walkMode))
  calcStepPattern, FIELD_WRAPPER(bool, false, nomadz_msgs::msg::PlannedSteps::is_left_phase, isLeftPhase),
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::PlannedSteps::speed, speed),
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::PlannedSteps::current_speed, currentSpeed),
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::PlannedSteps::max_speed, maxSpeed),

    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::PlannedSteps::step_duration, stepDuration),
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::WalkGeneratorData::upcoming_odometry_offset, upcomingOdometryOffset),
    FIELD_WRAPPER_DEFAULT(std::vector<UpcomingStep>, nomadz_msgs::msg::PlannedSteps::upcoming_steps, upcomingSteps),
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::PlannedSteps::next_left_steps, nextLeftStep),
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::PlannedSteps::next_right_steps, nextRightStep),

    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::PlannedSteps::measured_left_steps, measuredLeftStep),
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::PlannedSteps::measured_right_steps, measuredRightStep),
    FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::PlannedSteps::max_foot_height, maxFootHeight),
});
