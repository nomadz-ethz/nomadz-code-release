/**
 * @file KickTo.h
 *
 * Walk to the ball and kick it towards some position on the field, as quickly as possible.
 * On the way to the ball, walks around other players to avoid collision.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(KickTo, Vector2<> target, Pose2D tol = {fromDegrees(7.5f), 30.f, 20.f}, bool unused = false) {

  const float expectedKickLen = 3000.f;
  Vector2<> ballPositionEstimate;
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
    kickOffset = kickOffsetLeftFieldSide;
  } else if (gloBall.y < -1000) {
    kickOffset = kickOffsetRightFieldSide;
  } else {
    kickOffset = kickOffsetMiddleOfField;
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
      if ((theBallModel.estimate.position - ballPositionEstimate).abs() > 200 && theKickEngineOutput.isLeavingPossible) {
        goto kickPostBreak;
      }
      if (state_time > 3000) {
        goto done;
      }
    }
    action {
      const bool leftKick = (theBallModel.estimate.position.y >= 0);
      PowerKick(leftKick);
    }
  }

  state(kickPostBreak) {
    transition {
      if (state_time > 500) {
        goto done;
      }
    }
    action { StandWide(); }
  }

  target_state(done) {
    transition {
      if (state_time > 200) {
        goto walk;
      }
    }
    action { Stand(); }
  }
}
