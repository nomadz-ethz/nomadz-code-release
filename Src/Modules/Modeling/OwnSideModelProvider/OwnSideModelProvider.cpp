/**
 * @file OwnSideModelProvider.cpp
 *
 * The file implements a module that determines whether the robot cannot have left its own
 * side since the last kick-off.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "OwnSideModelProvider.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Core/Debugging/Debugging.h"
#include "Core/Streams/InStreams.h"

OwnSideModelProvider::OwnSideModelProvider()
    : lastPenalty(PENALTY_NONE), lastGameState(STATE_INITIAL), distanceWalkedAtKnownPosition(0),
      largestXPossibleAtKnownPosition(0), manuallyPlaced(false), timeWhenPenalized(0),
      gameStateWhenPenalized(STATE_INITIAL) {}

void OwnSideModelProvider::update(OwnSideModel& ownSideModel) {
  if (theGameInfo.state == STATE_SET && !theGroundContactState.contact) {
    manuallyPlaced = true;
  }

  // Unpenalized -> Penalized
  if (lastPenalty == PENALTY_NONE && theRobotInfo.penalty != PENALTY_NONE) {
    timeWhenPenalized = theFrameInfo.time;
    gameStateWhenPenalized = theGameInfo.state;
  }

  ownSideModel.returnFromGameControllerPenalty = false;
  ownSideModel.returnFromManualPenalty = false;

  if (theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT) {
    // Initial -> (any state)
    if (lastGameState == STATE_INITIAL && theGameInfo.state != STATE_INITIAL) {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = largestXInInitial;
    }
    // Manually penalized -> Unpenalized
    else if (lastPenalty == PENALTY_MANUAL && theRobotInfo.penalty == PENALTY_NONE) {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = -theFieldDimensions.centerCircleRadius - awayFromLineDistance;
      ownSideModel.returnFromManualPenalty = true;
    }
    // Penalized -> Unpenalized
    else if (lastPenalty != PENALTY_NONE && theRobotInfo.penalty == PENALTY_NONE &&
             (theFrameInfo.getTimeSince(timeWhenPenalized) > minPenaltyTime ||
              theGameInfo.state != gameStateWhenPenalized)) {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnPenaltyMark;
      ownSideModel.returnFromGameControllerPenalty = true;
    }
    // Set -> Playing
    else if (lastGameState == STATE_SET && theGameInfo.state == STATE_PLAYING) {
      distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
      if (manuallyPlaced) {
        if (theRobotInfo.number == TeamMateData::firstPlayer) {
          largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnGroundline;
        } else if (theGameInfo.kickingTeam == theOwnTeamInfo.teamColor) {
          largestXPossibleAtKnownPosition = -theFieldDimensions.centerCircleRadius - awayFromLineDistance;
        } else {
          largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnPenaltyArea + awayFromLineDistance;
        }
      } else if (theRobotInfo.number == TeamMateData::firstPlayer) {
        largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnPenaltyArea;
      } else {
        largestXPossibleAtKnownPosition = -awayFromLineDistance;
      }
    }
  }
  // Penalty shootout: Set
  else if (lastGameState == STATE_SET) {
    distanceWalkedAtKnownPosition = theOdometer.distanceWalked;
    if (theGameInfo.kickingTeam == theOwnTeamInfo.teamColor) {
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosOpponentPenaltyMark - 1000.f;
    } else {
      largestXPossibleAtKnownPosition = theFieldDimensions.xPosOwnGroundline;
    }
  }
  ownSideModel.largestXPossible = largestXPossibleAtKnownPosition + distanceUncertaintyOffset +
                                  (theOdometer.distanceWalked - distanceWalkedAtKnownPosition) * distanceUncertaintyFactor;
  ownSideModel.stillInOwnSide = ownSideModel.largestXPossible < 0;
  lastPenalty = theRobotInfo.penalty;
  lastGameState = theGameInfo.state;

  if (theGameInfo.state != STATE_SET) {
    manuallyPlaced = false;
  }
}

MAKE_MODULE(OwnSideModelProvider, Modeling)
