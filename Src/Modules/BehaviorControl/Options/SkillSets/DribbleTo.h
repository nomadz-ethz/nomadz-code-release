/**
 * @file DribbleTo.h
 *
 * Run behind ball, and if facing +/- 60 degrees towards opponent side, run into ball and do in walk kicks and shit.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

// tolRight = NAN: set to same as tolLeft
option(DribbleTo, Vector2<> target, float tolLeft = fromDegrees(30.f), float tolRight = NAN) {
  // const Vector2<> opponentGoal(theFieldDimensions.xPosOpponentGroundline, 0.f);

  if (isnan(tolRight)) {
    tolRight = tolLeft;
  }

  // const Vector2<> target = opponentGoal;
  const Vector2<> gloBall = theRobotPose * theBallModel.estimate.position;

  const Pose2D targetFrame((target - gloBall).angle(), gloBall);
  const Pose2D deviation = theRobotPose - targetFrame; // Own pose inside target frame

  const bool withinCone =
    (deviation.translation.angle() > fromDegrees(180.f) - tolRight)    // left side of rear cone, right side of forward cone
    || (deviation.translation.angle() < tolLeft - fromDegrees(180.f)); // right side of rear cone, left side of forward cone

  const float largeConeFactor = 1.05f; // 5% larger cone for hysteresis; must be > 1.0f
  const bool withinLargerCone =
    (deviation.translation.angle() >
     fromDegrees(180.f) - tolRight * largeConeFactor) // left side of rear cone, right side of forward cone
    || (deviation.translation.angle() <
        tolLeft * largeConeFactor - fromDegrees(180.f)); // right side of rear cone, left side of forward cone

  const float danger_distance = 600.f; // change ball position is a robot is closer than this, laterally

  initial_state(walkToCone) {
    transition {
      if (withinCone) {
        goto chase;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      GetBehindBall(target, Vector2<>(180.f, 0.f));
    }
  }

  state(chase) {
    transition {
      if (theBallModel.timeSinceLastSeen < 2000.f) {
        const auto& ballPos = theBallModelAfterPreview.estimate.position;
        if (ballPos.x < 220.f && std::abs(ballPos.y) < 80.f) {
          goto frontKick;
        }
      }
      if (!withinLargerCone) {
        goto walkToCone;
      }
      if (theBallModel.timeSinceLastSeen > 2000.f && theBallModel.estimate.position.x < 300.f) {
        goto walkBackwards;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      if (theBallModel.estimate.position.abs() > 300.f) {
        float angle = (gloBall - theRobotPose.translation).angle();
        TravelTo({angle, gloBall});
      } else {
        WalkToTarget(Pose2D(1.f, 1.f, 0.4f),
                     Pose2D(theBallModelAfterPreview.estimate.position.angle(),
                            theBallModelAfterPreview.estimate.position + Vector2<>(40.f, 10.f)));
      }
    }
  }

  state(walkBackwards) {
    transition {
      if (state_time > 5000.f || theBallModel.timeSinceLastSeen < 500.f) {
        goto chase;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkAtSpeed(Pose2D(0.f, -100.f, 0.f));
    }
  }

  state(frontKick) {
    transition {
      if (action_done || state_time > 500) {
        goto chase;
      }

      // if (libCodeRelease.nearestPlayerInSight(fromDegrees(40.f), danger_distance) < 500.f){
      //   goto trickBall;
      // }
    }
    action {
      WalkToTarget(Pose2D(1.f, 1.f, 0.4f),
                   Pose2D(theBallModelAfterPreview.estimate.position.angle(),
                          theBallModelAfterPreview.estimate.position + Vector2<>(40.f, 10.f)));
      if (theBallModelAfterPreview.estimate.position.y > 0.f) {
        InWalkKick(WalkRequest::frontKickLeft);
      } else {
        InWalkKick(WalkRequest::frontKickRight);
      }
    }
  }

  state(trickBall) {
    transition {
      if (state_time > 1000.f) {
        goto chase;
      }
    }
    action {
      if (nearestPlayerAngle(fromDegrees(20.f), danger_distance) > 0.f) {
        OmniKick(-pi_4, 0.1f);
      } else {
        OmniKick(pi_4, 0.1f);
      }
    }
  }
}

float nearestPlayerAngle(float angle, float offset) {
  auto& players = thePlayerModel.players;
  float minDistance = INFINITY;
  float minAngle = 0.f;
  for (auto& player : players) {
    auto& localPos = player.relPosOnField;
    if (localPos.x <= 20) {
      continue;
    }

    if (std::abs(localPos.angle()) < angle && std::abs(localPos.y) < offset) {
      minDistance = std::min(minDistance, localPos.abs());
      minAngle = localPos.angle();
    }
  }
  return minAngle;
}
