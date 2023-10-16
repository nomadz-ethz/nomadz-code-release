/**
 * @file PlannedSteps.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <deque>
#include "Core/Function.h"
#include "Core/Math/Pose2D.h"
#include "Core/RingBuffer.h"
#include "Core/Streams/FieldWrapper.h"
#include "Representations/MotionControl/WalkGeneratorData.h"
#include "Representations/Sensing/FootSupport.h"
#include "MotionRequest.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/planned_steps.hpp"
#endif

STREAMABLE_DECLARE(PlannedSteps)

STREAMABLE_ROS(PlannedSteps, {
public:
  FUNCTION(void()) reset;
  FUNCTION(void(WalkGeneratorData & generator, Pose2D & speed, Pose2D & target, WalkGeneratorData::WalkMode walkMode))
  calcStepPattern, FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::PlannedSteps::speed, speed),
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::PlannedSteps::current_planned_speed, currentPlannedSpeed),
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::PlannedSteps::max_speed, maxSpeed),

    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::PlannedSteps::step_duration, stepDuration),
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::PlannedSteps::upcoming_odometry_offset, upcomingOdometryOffset),
    FIELD_WRAPPER_DEFAULT(std::vector<Leg>, nomadz_msgs::msg::PlannedSteps::upcoming_steps, upcomingSteps),

    FIELD_WRAPPER_DEFAULT(Leg, nomadz_msgs::msg::PlannedSteps::current_executed_step, currentExecutedStep),
    FIELD_WRAPPER_DEFAULT(Leg, nomadz_msgs::msg::PlannedSteps::prev_executed_step, prevExecutedStep),

    FIELD_WRAPPER_DEFAULT(Pose2D[Leg::numOfSides], nomadz_msgs::msg::PlannedSteps::next_step, nextStep),
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::PlannedSteps::next_origin, nextOrigin),

    FIELD_WRAPPER_DEFAULT(Pose2D[Leg::numOfSides], nomadz_msgs::msg::PlannedSteps::measured_step, measuredStep),

    FIELD_WRAPPER_DEFAULT(bool, nomadz_msgs::msg::PlannedSteps::is_leaving_possible, isLeavingPossible),
    FIELD_WRAPPER_DEFAULT(bool, nomadz_msgs::msg::PlannedSteps::is_current_step_kick, isCurrentStepKick),
    FIELD_WRAPPER_DEFAULT(bool, nomadz_msgs::msg::PlannedSteps::is_current_step_prepare, isCurrentStepPrepare),
});
