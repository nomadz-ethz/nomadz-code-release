/**
 * @file Defender2v2.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iostream>
#include <vector>

option(Defender2v2) {

  // global x limit for the DefensivePlay2v2
  float hardXLimit = 25.f;
  float takeBallXLimit = 0.f;
  const Vector2<> gloBall = relToGlo(theBallModel.estimate.position);

  bool wantsBall =
    relToGlo(theBallModel.estimate.position).x < takeBallXLimit && libCodeRelease.timeSinceBallWasSeen() < 1500.f;

  const Pose2D defensive_recovery = {0, {static_cast<float>(theFieldDimensions.xPosOwnGroundline * 0.8), 0}};

  common_transition {
    if (theRobotPose.timeSinceLastValid > 5000.f) {
      goto relocate;
    }
    if (libWorldModel.timeSinceBallLastSeen() > 6000.f) {
      goto search;
    }
    if (theBallModel.timeSinceLastSeen < 1500.f && libCodeRelease.hasBallLock(wantsBall, 0)) {
      goto defend;
    }
  }

  initial_state(search) {
    transition {
      if (action_aborted) {
        goto relocate;
      }
      if (libWorldModel.timeSinceBallLastSeen() < 2500.f || action_done) {
        goto defend;
      }
    }
    action { DefenderSearch(); }
  }

  state(relocate) {
    transition {
      if (action_done) {
        goto search;
      }
    }
    action { Relocate(); }
  }

  state(defend) {
    transition {
      // If the ball was not seen after a certain time and no search is in progress search for ball
      if (action_aborted) {
        goto search;
      }
      if (!libCodeRelease.hasBallLock(wantsBall, 0) || theBallModel.timeSinceLastSeen > 1500.f) {
        if (libWorldModel.timeSinceBallLastSeen() < 2500.f) {
          goto defendPosition;
        } else {
          goto search;
        }
      }
      if (gloBall.x >= 0) {
        goto defendPosition;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      DefensivePlay2v2(hardXLimit);
    }
  }

  state(defendPosition) {
    transition { ; }
    action { DefenderPosition(); }
  }
}
