/**
 * @file TeamInfo.h
 *
 * The file declares a class that encapsulates the structure TeamInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Core/Communication/RoboCupControlData.h"
#include "Core/Streams/AutoStreamable.h"
#include "Representations/Infrastructure/RobotInfo.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/team_info.hpp"
#endif
STREAMABLE_DECLARE(TeamInfo)

STREAMABLE_ROS(TeamInfo, {
public:
  /** Update data from RoboCup GameCtrl data. */
  void setFromRoboCupData(RoboCup::TeamInfo * teamInfo) {
    teamNumber = teamInfo->teamNumber;
    teamColor = teamInfo->teamColor;
    goalkeeperColor = teamInfo->goalkeeperColour;
    score = teamInfo->score;
    messageBudget = teamInfo->messageBudget;
    for (int i = 0; i < MAX_NUM_PLAYERS; ++i) {
      players[i].setFromRoboCupData(&teamInfo->players[i]);
    }
  }

  /** Draws the score in the scene view. */
  virtual void draw() const;
  , FIELD_WRAPPER(uint8_t, 0, nomadz_msgs::msg::TeamInfo::team_number, teamNumber), /**< Number of the team. */
    FIELD_WRAPPER(uint8_t, 0, nomadz_msgs::msg::TeamInfo::team_color, teamColor),   /**< Color of the team. */
    FIELD_WRAPPER(
      uint8_t, 0, nomadz_msgs::msg::TeamInfo::goalkeeper_color, goalkeeperColor), /**< Color of the goalkeeper. */
    FIELD_WRAPPER(uint8_t, 0, nomadz_msgs::msg::TeamInfo::score, score),          /**< Score of the team. */
    FIELD_WRAPPER(
      uint16_t, 1200, nomadz_msgs::msg::TeamInfo::message_budget, messageBudget), /**< Message budget of the team. */
    FIELD_WRAPPER_DEFAULT(
      RobotInfo[MAX_NUM_PLAYERS], nomadz_msgs::msg::TeamInfo::players, players), /**< Players in the team. */

    for (int i = 0; i < MAX_NUM_PLAYERS; ++i) {
    players[i].number = i + 1;
  }
});

STREAMABLE_ALIAS(OwnTeamInfo, TeamInfo, {
public:
  ALIAS_ROS_HELPERS(OwnTeamInfo, TeamInfo)
  OwnTeamInfo();
  virtual void draw() const override;
});

STREAMABLE_ALIAS(OpponentTeamInfo, TeamInfo, {
public:
  OpponentTeamInfo();
});
