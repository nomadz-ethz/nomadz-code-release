/**
 * @file Striker.h
 *
 * Striker state machine
 *
 *               ----------
// *             | search |  <---  timeSinceBallWasSeen() > 5000 <-|
 *               ----------                                               |
 *                     |                                                  |
 *                     | timeSinceBallWasSeen() < 300                     |
 *                    \|/                                                 |
 *    -------       ------                                                |
 *   | Start | <-> | play |-----------------------------------------------|
 *    -------       ------
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iostream>
#include <vector>

option(Striker) {

  common_transition {
    if (theGameInfo.setPlay == SET_PLAY_GOAL_KICK || theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK ||
        theGameInfo.setPlay == SET_PLAY_CORNER_KICK || theGameInfo.setPlay == SET_PLAY_KICK_IN) {
      goto freeKick;
    } else if (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK) {
      goto penaltyKick;
    }
  }

  initial_state(start) {
    transition {
      if (state_time > 500) {
        if (initial_pass == true && (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam)) {
          goto firstPass;
        } else {
          goto play;
        }
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }

  state(searchAndRelocate) {
    transition {
      if (action_done) {
        goto play;
      }
    }
    action { SearchAndRelocate(); }
  }

  /* state(search) {
    transition {
      if (action_done) {
        goto play;
      }
      if (action_aborted) {
        goto relocate;
      }
    }
    action { StrikerSearch(); }
  } */

  state(play) {
    transition {
      // If the ball was not seen after a certain time and no search is in progress search for ball
      if (action_aborted) {
        goto searchAndRelocate;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      OffensivePlay();
    }
  }

  state(firstPass) {
    transition {
      if (action_done || action_aborted || libCodeRelease.timeSinceBallWasSeen() > 1500 ||
          abs(relToGlo(theBallModel.estimate.position).x) > 800.f ||
          abs(relToGlo(theBallModel.estimate.position).y) > 800.f) {
        goto play;
      }
      if (!libCodeRelease.hasBallLock(true)) {
        goto play;
      }
      action {
        initial_pass = false;
        Vector2<> target = {1000, 500};
        DribbleTo(target);
      }
    }
  }

  state(freeKick) {
    transition {
      if (theGameInfo.setPlay == SET_PLAY_NONE) {
        goto searchAndRelocate;
      }
    }

    action {
      const Vector2<> posGoalFreeKickStriker =
        (theRobotPose.translation.y < 0) ? posGoalFreeKickStrikerOne : posGoalFreeKickStrikerTwo;
      OffensiveFreeKick({1.57f, posGoalFreeKickStriker},
                        {rotCornerFreeKickStriker, posCornerFreeKickStriker},
                        {rotKickInStriker, posKickInStriker});
    }
  }

  state(penaltyKick) {
    transition {
      if (theGameInfo.setPlay == SET_PLAY_NONE) {
        goto searchAndRelocate;
      }
    }
    action {
      if (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam) {
        PenaltyStriker();
      } else {
        StandHigh();
      }
    }
  }
}
