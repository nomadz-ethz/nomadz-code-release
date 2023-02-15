/**
 * @file GoalSymbols.h
 *
 * The file declares a class that containts data about the current behavior state.
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is Oliver Urbann
 */

#pragma once
#include "Core/Streams/AutoStreamable.h"

/**
 * \class GoalSymbols
 * A class that containts data about the current behavior state.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/goal_symbols.hpp"
#endif
STREAMABLE_DECLARE(GoalSymbols)

STREAMABLE_ROS(
  GoalSymbols,
  {
    ,
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::GoalSymbols::opening_angle_of_opp_goal, openingAngleOfOppGoal),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::GoalSymbols::center_angle_to_own_goal, centerAngleToOwnGoal),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::GoalSymbols::center_angle_ball_to_opp_goal_w_c, centerAngleBallToOppGoalWC),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::GoalSymbols::left_angle_ball_to_opp_goal_w_c, leftAngleBallToOppGoalWC),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::GoalSymbols::right_angle_ball_to_opp_goal_w_c, rightAngleBallToOppGoalWC),
    FIELD_WRAPPER(float,
                  0.f,
                  nomadz_msgs::msg::GoalSymbols::center_angle_to_opp_goal_as_seen_from_ball,
                  centerAngleToOppGoalAsSeenFromBall),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::GoalSymbols::smallest_ball_to_goal_post_angle, smallestBallToGoalPostAngle),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::GoalSymbols::approach_angle_w_c, approachAngleWC),
  });
