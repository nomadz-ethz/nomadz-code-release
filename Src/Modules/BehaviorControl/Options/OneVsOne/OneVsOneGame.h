/**
 * @file OneVsOneGame.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iostream>
#include <vector>

option(OneVsOneGame) {

  const Pose2D startPose =
    Pose2D(0.f, (theFieldDimensions.xPosOwnGroundline + theFieldDimensions.xPosOwnPenaltyArea) / 2.0, 0.0f);

  common_transition {
    if (theGameInfo.state == STATE_INITIAL) {
      goto initial;
    } else if (theGameInfo.state == STATE_FINISHED) {
      goto finished;
    } else if (theFallDownState.state == FallDownState::onGround) {
      goto getUp;
    } else if (theGameInfo.state == STATE_READY) {
      goto ready;
    } else if (theGameInfo.state == STATE_SET) {
      goto set;
    } else if (theGameInfo.state == STATE_PLAYING) {
      goto playing;
    }
  }

  state(getUp) {
    action { GetUp(); }
  }

  initial_state(initial) {
    action {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }

  state(finished) {
    action {
      theHeadControlMode = HeadControl::lookForward;
      StandHigh();
    }
  }

  state(ready) {
    action {
      theHeadControlMode = HeadControl::lookForward;
      TravelTo(startPose);
    }
  }

  state(set) {
    action {
      if (libCodeRelease.timeSinceBallWasSeen() < 1000.f) {
        theHeadControlMode = HeadControl::lookAtBall;
      } else {
        theHeadControlMode = HeadControl::scanLeftRight;
      }
      StandHigh();
    }
  }

  state(playing) {
    action { OneVsOnePlay(); }
  }
}
