/**
 * @file PersonalData.h
 *
 * Declaration of a class representing information about the Robot.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once
#include "Core/Streams/AutoStreamable.h"
#include "Representations/Modeling/BallAfterKickPose.h"
#include "Core/Enum.h"
#include "Core/Math/Vector2.h"
#include <string.h>

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/personal_data.hpp"
#endif
STREAMABLE_DECLARE(PersonalData)

STREAMABLE_ROS(PersonalData, {
public:
  ENUM(BallLockState, NO_LOCK, REQ_LOCK, HAS_LOCK);
  // Determines if the player wants a ball and (if yes) what score it assigns
  , FIELD_WRAPPER(bool, false, nomadz_msgs::msg::PersonalData::sync_team_required, syncTeamRequired),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::PersonalData::has_ball_lock, hasBallLock),
    FIELD_WRAPPER(BallLockState, NO_LOCK, nomadz_msgs::msg::PersonalData::ball_lock_state, ballLockState),
    FIELD_WRAPPER(float, INFINITY, nomadz_msgs::msg::PersonalData::ball_score, ballScore),
    FIELD_WRAPPER(bool,
                  false,
                  nomadz_msgs::msg::PersonalData::should_approach_ball,
                  shouldApproachBall), // Just approach the ball. Not engage yet
    FIELD_WRAPPER_DEFAULT(
      BallLockState[8], nomadz_msgs::msg::PersonalData::team_mate_ball_lock_states, teamMateBallLockStates),
    FIELD_WRAPPER_DEFAULT(float[8], nomadz_msgs::msg::PersonalData::team_mate_ball_scores, teamMateBallScores),
});

/**
 * @class PersonalDataCompressed
 * A compressed version of PersonalData used in team communication
 */
STREAMABLE_DECLARE_LEGACY(PersonalDataCompressed)
STREAMABLE_LEGACY(PersonalDataCompressed, {
public:
  PersonalDataCompressed(const PersonalData& personalData);
  operator PersonalData() const,

    (char)ballLockState, (float)ballScore,
});
