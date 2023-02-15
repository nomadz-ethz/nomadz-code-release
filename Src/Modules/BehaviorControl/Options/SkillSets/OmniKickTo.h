/**
 * @file OmniKickTo.h
 *
 * Walk to the ball and kick it towards some position on the field, as quickly as possible.
 * On the way to the ball, walks around other players to avoid collision.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(OmniKickTo, Vector2<> target, Pose2D tol = {fromDegrees(7.5f), 30.f, 20.f}, bool isPassKick = false) {

  const float maxStraightStrength = 1.f;
  float expectedKickLen = 3000.f;
  float kickStrength = maxStraightStrength;

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
  Vector2<> ballToTarget = (target - gloBall);

  Rangef strengthRange = Rangef::ZeroOneRange();

  if (isPassKick) {
    // float u_r = 0.05f;
    // float c1 = -0.0427;
    // float c2 = 1.0762;
    // float c3 = -0.0845;

    // float kickVelocity = sqrt(2 * u_r * 9.81 * ballToTarget.abs());
    // kickStrength = c1 + c2 * kickVelocity + c3 * sqr(kickVelocity);
    // expectedKickLen = ballToTarget.abs();
    // OUTPUT_TEXT("kickVeclocuty = " << kickVelocity << " kickStrength = " << kickStrength);
    float p1 = -1.14904987553885e-07;
    float p2 = 0.000796416865637127;
    float p3 = -0.336600533590782;

    expectedKickLen = ballToTarget.abs();
    kickStrength = strengthRange.limit(p1 * sqr(expectedKickLen) + p2 * expectedKickLen + p3);
    // OUTPUT_TEXT("passDist = " << expectedKickLen << " kickStrength = " << kickStrength);
  }

  bool usingDefault = kickStrength > switchThreshold;
  Vector2<> kickOffset = usingDefault ? kickOffsetOmniKick : kickOffsetOmniKickLong;

  initial_state(walk) {
    transition {
      if (action_done) {
        thePersonalData.lastKickTime = static_cast<float>(theFrameInfo.time);
        thePersonalData.kickEstimate = gloBall + ballToTarget.normalize() * expectedKickLen;
        goto kick;
      }
    }
    action { OmniGetBehindBall(target, kickOffset, usingDefault, tol); }
  }

  state(kick) {
    transition {
      if (state_time > 3000) {
        goto done;
      }
    }
    action {
      // const bool leftKick = (theBallModel.estimate.position.y >= 0);
      const float kickAngle = angleDifference(theRobotPose.rotation, ballToTarget.angle());
      OmniKick(kickAngle, kickStrength, usingDefault);
      // PowerKick(leftKick);
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
