/**
 * @file TestRole.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

Pose2D testRoleTarget;

option(TestRole) {

  common_transition {
    // nope
  }

  initial_state(start) {
    transition {}
    action {
      theHeadControlMode = HeadControl::walkScan;
      Stand();
    }
  }

  state(walkToBall) {
    transition {
      if (theBallModelAfterPreview.estimate.position.x < 220.f &&
          std::abs(theBallModelAfterPreview.estimate.position.y) < 80.f) {
        goto powerKick;
      } else if (theBallModelAfterPreview.estimate.position.x < 0.f) {
        goto walkBackToBall;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToTarget(Pose2D(1.f, 1.f, 0.2f),
                   Pose2D(theBallModelAfterPreview.estimate.position.angle(),
                          theBallModelAfterPreview.estimate.position + Vector2<>(40.f, 0.f)));
    }
  }

  state(kickStep) {
    transition {
      if (state_time > 500) {
        goto walkToBall;
      }
    }
    action {
      WalkToTarget(Pose2D(1.f, 1.f, 0.2f),
                   Pose2D(theBallModelAfterPreview.estimate.position.angle(),
                          theBallModelAfterPreview.estimate.position + Vector2<>(40.f, 0.f)));
      if (theBallModelAfterPreview.estimate.position.y > 0.f) {
        theMotionRequest.walkRequest.stepRequest = WalkRequest::frontKickLeft;
      } else {
        theMotionRequest.walkRequest.stepRequest = WalkRequest::frontKickRight;
      }
    }
  }

  state(walkBackToBall) {
    transition {
      if (theBallModelAfterPreview.estimate.position.x > 80.f) {
        goto walkToBall;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkAtSpeed(Pose2D(0.f, 200.f, 0.f));
    }
  }

  state(walkToTargetBall) {
    transition {
      if (theBallModel.lost) {
        goto waitForBall;
      }
    }
    action {
      Vector2<> goal(theFieldDimensions.xPosOpponentGroundline, 0.f);
      Vector2<> ball(relToGlo(theBallModel.estimate.position));
      Vector2<> ballToGoal = goal - ball;
      Vector2<> directionToGoal = ballToGoal / ballToGoal.abs();
      Vector2<> directionLeft = directionToGoal;
      directionLeft.rotateLeft();
      Pose2D target(ballToGoal.angle(), ball - directionToGoal * 150.f - directionLeft * 50.f);

      float dx = target.translation.x - theRobotPose.translation.x;
      float dy = target.translation.y - theRobotPose.translation.y;
      float dr = target.rotation - theRobotPose.rotation;
      if (-10.f < dx && dx < 40.f && -25.f < dy && dy < 25.f && std::abs(dr) < fromDegrees(5.f)) {
        Stand();
      } else {
        WalkTo(target);
      }

      theHeadControlMode = HeadControl::lookAtBall;
    }
  }

  state(walk) {
    transition {
      const float distanceThreshold = 50.f;
      const float rotationThreshold = fromDegrees(3.f);
      if ((theRobotPose.translation - testRoleTarget.translation).abs() < distanceThreshold &&
          std::abs(theRobotPose.rotation - testRoleTarget.rotation) < rotationThreshold) {
        goto rest;
      }
    }
    action { WalkTo(testRoleTarget); }
  }

  state(rest) {
    transition {
      const float restTime = 3000.f; // ms
      if (state_time > restTime && theBallModel.valid) {
        testRoleTarget.translation.x =
          ((float)rand() / (float)RAND_MAX * 2.f - 1.f) * theFieldDimensions.xPosOpponentGroundline;
        testRoleTarget.translation.y = ((float)rand() / (float)RAND_MAX * 2.f - 1.f) * theFieldDimensions.yPosLeftSideline;
        testRoleTarget.rotation = ((float)rand() / (float)RAND_MAX * 2.f - 1.f) * fromDegrees(180.f);
        goto walk;
      }
    }
    action { Stand(); }
  }

  state(walkToBallProp) {
    transition {
      if (theBallModel.lost) {
        goto waitForBall;
      }
    }

    action {
      float speedR = 0.f;
      float speedX = 0.f;
      float speedY = 0.f;

      const float speedRLim[2] = {-1.f, 1.f};
      const float speedXLim[2] = {0.f, 95.f};

      const float pX = 95.f / 300.f;
      const float pR = 1.f / 1.f;
      const float kXR = 30.f / 1.f;
      const float threshDR = 200.f;

      float dx = theBallModel.estimate.position.x + 140.f;
      float dy = theBallModel.estimate.position.y - 50.f;
      float dr = atanf(dy / dx);

      speedR = pR * dr *
               std::min(std::sqrt(dx * dx + dy * dy) / threshDR + 0.2f,
                        1.0f); // pR*dr, unless closer than threshDR from target
      speedR = std::max(std::min(speedR, speedRLim[1]), speedRLim[0]);
      speedX = std::max(std::min(pX * dx, speedXLim[1] - kXR * speedR), speedXLim[0]);
      WalkAtSpeed(Pose2D(speedR, speedX, speedY));

      theHeadControlMode = HeadControl::lookAtBall;
    }
  }

  state(waitForBall) {
    transition {
      if (theBallModel.valid) {
        goto walkToTargetBall;
      }
    }
    action {
      theHeadControlMode = HeadControl::scanLeftRight;
      Stand();
    }
  }
  state(powerKick) {
    transition {
      if (state_time > 3000) {
        goto walkToBall;
      }
    }
    action { PowerKick(); }
  }
}
