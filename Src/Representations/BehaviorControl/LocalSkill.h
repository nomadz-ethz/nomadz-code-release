/**
 * @file LocalSkill.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Function.h"
#include "Core/Math/Pose2D.h"
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/local_skill.hpp"
#endif

STREAMABLE_DECLARE(LocalSkill)

STREAMABLE_ROS(LocalSkill, {
public:
  ENUM(Skill, none, dribble, inWalkKick, fastKick);
  ENUM(TargetProperty, freeRange, goalCenter, teamMate);
  , FIELD_WRAPPER(Skill, none, nomadz_msgs::msg::LocalSkill::skill_type, skillType),
    FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::LocalSkill::next_target_ball_angle, nextTargetBallAngle),
    FIELD_WRAPPER_DEFAULT(TargetProperty, nomadz_msgs::msg::LocalSkill::target_property, targetProperty),
    FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::LocalSkill::best_free_ranges_min, bestFreeRangesMin),
    FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::LocalSkill::best_free_ranges_max, bestFreeRangesMax),

  // desired ball position in the robot frame
});
