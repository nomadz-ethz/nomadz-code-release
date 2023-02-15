/**
 * @file KickTo2v2.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(KickTo2v2, Vector2<> target, Pose2D tol = {fromDegrees(7.5f), 30.f, 20.f}) {

  const float expectedKickLen = 3000.f;

  // Default tolerances
  if (tol.rotation <= 0.f) {
    tol.rotation = fromDegrees(7.5f);
  }
  if (tol.translation.x <= 0.f) {
    tol.translation.x = 30.f;
  }
  if (tol.translation.y <= 0.f) {
    tol.translation.y = 20.f;
  }

  const Vector2<> gloBall = relToGlo(theBallModel.estimate.position);
  Vector2<> kickOffset;
  if (gloBall.y > 1000) {
    kickOffset = kickOffsetPass;
  } else if (gloBall.y < -1000) {
    kickOffset = kickOffsetPass;
  } else {
    kickOffset = kickOffsetPass;
  }

  initial_state(walk) {
    transition {
      if (action_done) {
        thePersonalData.lastKickTime = static_cast<float>(theFrameInfo.time);
        Vector2<> direction = (target - gloBall);
        direction.normalize();
        thePersonalData.kickEstimate = gloBall + direction * expectedKickLen;
        goto kick;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      GetBehindBall(target, kickOffset, tol);
    }
  }

  state(kick) {
    transition {
      if (state_time > 3000) {
        goto done;
      }
    }
    action {
      const bool leftKick = (theBallModel.estimate.position.y >= 0);
      PassKick(leftKick);
    }
  }

  target_state(done) {
    transition {
      if (state_time > 100) {
        goto walk;
      }
    }
    action { Stand(); }
  }
}
