/**
 * @file OneVsOnePlay.h
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iostream>
#include <vector>

option(OneVsOnePlay) {

  const Pose2D baseKickTol = {fromDegrees(7.f), 30.f, 20.f};
  Vector2<> ballPos = theRobotPose * theBallModel.estimate.position;

  common_transition {
    if (theRobotPose.timeSinceLastValid > 5000.f) {
      goto relocate;
    }
  }

  initial_state(wait) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() < 1000.f && ballPos.x < -25.f) {
        goto play;
      }
    }
    action { OneVsOneWait(); }
  }

  state(relocate) {
    transition {
      if (action_done) {
        goto wait;
      }
    }
    action { Relocate(); }
  }

  state(play) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() > 2000.f || ballPos.x > -25.f) {
        goto wait;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      Vector2<> kickTarget = {theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal};
      KickTo(kickTarget, baseKickTol);
    }
  }
}
