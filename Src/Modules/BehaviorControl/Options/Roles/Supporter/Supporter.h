/**
 * @file Supporter.h
 *
 *  Striker state machine
 *
 *               ----------
// *             | search |  <---  timeSinceBallWasSeen() > 5000 <-|
 *               ----------                                               |
 *                     |                                                  |
 *                     | timeSinceBallWasSeen() < 300                     |
 *                    \|/                                                 |
 *    -------       ------                                               |
 *   | Start | <-> | play |-----------------------------------------------|
 *    -------       ------
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iostream>
#include <vector>

option(Supporter) {

  common_transition {
    if (theGameInfo.setPlay == SET_PLAY_GOAL_KICK || theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK ||
        theGameInfo.setPlay == SET_PLAY_CORNER_KICK || theGameInfo.setPlay == SET_PLAY_KICK_IN) {
      goto freekick;
    } else if (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK) {
      goto penaltyKick;
    }
  }

  initial_state(start) {
    transition {
      if (state_time > 500) {
        goto play;
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

  /*   state(search) {
      transition {
        if (action_done) {
          goto play;
        }
        if (action_aborted) {
          goto relocate;
        }
      }
      action { SupporterSearch(); }
    } */

  state(play) {
    transition {
      // If the ball was not seen after a certain time and no search is in progress search for ball
      if (action_aborted || libCodeRelease.timeSinceBallWasSeen() > ballLostTime ||
          theRobotPose.timeSinceLastValid > robotPoseValidTime) {
        goto searchAndRelocate;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;

      if (supporterCatchBall) {
        SupporterCoop();
      } else {
        OffensivePlay();
      }
    }
  }

  state(freekick) {
    transition {
      if (theGameInfo.setPlay == SET_PLAY_NONE) {
        goto searchAndRelocate;
      }
    }
    action {
      OffensiveFreeKick({1.57f, posGoalFreeKickSupporter},
                        {rotCornerFreeKickSupporter, posCornerFreeKickSupporter},
                        {rotKickInSupporter, posKickInSupporter});
    }
  }

  state(penaltyKick) {
    transition {
      if (theGameInfo.setPlay == SET_PLAY_NONE) {
        goto searchAndRelocate;
      }
    }
    action {
      if (!theTeamMateData.isActive[5]) {
        PenaltyStriker();
      } else {
        StandHigh();
      }
    }
  }
}
