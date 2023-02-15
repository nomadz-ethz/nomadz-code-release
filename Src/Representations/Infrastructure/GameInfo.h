/**
 * @file GameInfo.h
 *
 * The file declares a class that encapsulates the structure RoboCupGameControlData
 * defined in the file RoboCupGameControlData.h that is provided with the GameController.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Core/Communication/RoboCupControlData.h"
#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/game_info.hpp"
#endif
STREAMABLE_DECLARE(GameInfo)

STREAMABLE_ROS(GameInfo, {
public:
  /** Update data from RoboCup GameCtrl data. */
  void setFromRoboCupData(RoboCup::RoboCupGameControlData * gameCtrl) {
    competitionType = gameCtrl->competitionType;
    gamePhase = gameCtrl->gamePhase;
    state = gameCtrl->state;
    setPlay = gameCtrl->setPlay;
    firstHalf = gameCtrl->firstHalf;
    kickingTeam = gameCtrl->kickingTeam;
    secsRemaining = gameCtrl->secsRemaining;
  }

  /** Draws the game time in the scene view. */
  void draw() const;
  ,
    FIELD_WRAPPER(
      uint8_t, 0, nomadz_msgs::msg::GameInfo::competition_type, competitionType), /**< Type of the competition. */
    FIELD_WRAPPER(uint8_t, 0, nomadz_msgs::msg::GameInfo::game_phase, gamePhase), /**< Phase of the game. */
    FIELD_WRAPPER(uint8_t, 0, nomadz_msgs::msg::GameInfo::state, state),          /**< State of the game. */
    FIELD_WRAPPER(uint8_t, 0, nomadz_msgs::msg::GameInfo::set_play, setPlay),     /**< Active set play. */
    FIELD_WRAPPER(uint8_t, 1, nomadz_msgs::msg::GameInfo::first_half, firstHalf), /**< Equal to 1 in the first half. */
    FIELD_WRAPPER(uint8_t,
                  1,
                  nomadz_msgs::msg::GameInfo ::kicking_team,
                  kickingTeam), /**< Team number of the next team to do kick off or set play. */
    FIELD_WRAPPER(
      int16_t, 1, nomadz_msgs::msg::GameInfo ::secs_remaining, secsRemaining), /**< Number of seconds left in half. */
    FIELD_WRAPPER(unsigned,
                  0,
                  nomadz_msgs::msg::GameInfo::time_last_package_received,
                  timeLastPackageReceived),                                          /**< Determine if GC is active. */
    FIELD_WRAPPER(bool, true, nomadz_msgs::msg::GameInfo::ball_in_play, ballInPlay), /**< If ball is in play. */
});

/** Sent by GameController */
class RawGameInfo : public GameInfo {};
