/**
 * @file SearchAndRelocate.h
 *
 * Replacement for the previous relocation and ball search logic
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(SearchAndRelocate, bool ignoreBall = false) {
  float norminalDistance = (targetDistanceRange.min + targetDistanceRange.max) / 2;
  Angle relBallAngle = std::atan2(theBallModel.estimate.position.y, theBallModel.estimate.position.x);
  Vector2<> targetPosition =
    theBallModel.estimate.position - Vector2<>(cos(relBallAngle), sin(relBallAngle)) * norminalDistance;

  common_transition {}

  // Initial state just to play this sound on entry
  initial_state(enter) {
    transition {
      if (state_time > 2000) {
        goto ballLostNotRelocalized;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      Stand();
    }
  }

  state(ballLostNotRelocalized) {
    transition {
      if (ignoreBall || theCombinedWorldModel.ballIsValidOthers) {
        goto simpleRelocate;
      }
      if (theBallModel.valid) {
        goto approachBall;
      }
      if (theRobotPose.valid) {
        goto ballLostRelocalized;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      RelocateWhileSearchingBall();
    }
  }

  state(ballLostRelocalized) {
    transition {
      if (theBallModel.valid || theCombinedWorldModel.ballIsValidOthers) {
        goto success;
      }
      if (theRobotPose.lost) {
        goto ballLostNotRelocalized;
      }
    }
    action { AllSearch(); }
  }

  state(approachBall) {
    transition {
      if (theBallModel.lost) {
        goto ballLostNotRelocalized;
      }
      if (theRobotPose.valid) {
        goto success;
      }

      if (theBallModel.estimate.position.abs() < 800) {
        if (!shouldEngageBall) {
          goto observeBallAndRelocate;
        } else if (theBallModel.estimate.position.abs() < norminalDistance + 20) {
          goto hasBallNotReLocalized;
        }
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToTarget(Pose2D(1.f, 1.f, 1.f), Pose2D(relBallAngle, targetPosition));
    }
  }

  state(hasBallNotReLocalized) {
    transition {
      if (theBallModel.lost) {
        goto ballLostNotRelocalized;
      }
      if (theRobotPose.valid) {
        goto success;
      }
      if (theBallModel.estimate.position.abs() > targetDistanceRange.max) {
        goto approachBall;
      }
    }
    action { RelocateWhileProtectBall(); }
  }

  state(observeBallAndRelocate) {
    transition {
      if (theBallModel.lost) {
        goto ballLostNotRelocalized;
      }
      if (theRobotPose.valid) {
        goto success;
      }
      if (state_time > 2000 && shouldEngageBall) {
        goto approachBall;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      Stand();
    }
  }

  state(simpleRelocate) {
    transition {
      if (theRobotPose.valid) {
        goto success;
      }
    }
    action { RelocateWhileSearchingBall(); }
  }

  target_state(success) {
    action { theHeadControlMode = HeadControl::lookAtBall; }
  }
}
