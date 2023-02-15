/**
 * @file SearchAndRelocate.h
 *
 * Replacement for the previous relocation and ball search logic
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(SearchAndRelocate) {
  float norminalDistance = (targetDistanceRange.min + targetDistanceRange.max) / 2;
  Angle relBallAngle = std::atan2(theBallModel.estimate.position.y, theBallModel.estimate.position.x);
  Vector2<> targetPosition =
    theBallModel.estimate.position - Vector2<>(cos(relBallAngle), sin(relBallAngle)) * norminalDistance;

  common_transition {}

  // Initial state just to play this sound on entry
  initial_state(enter) {
    transition {
      if (action_done) {
        goto ballLostNotRelocalized;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      PlaySound("lost.wav");
    }
  }

  state(ballLostNotRelocalized) {
    transition {
      if (theBallModel.timeSinceLastSeen < ballValidTime) {
        goto approachBall;
      }
      if (theBallModel.timeSinceLastSeen > ballLostTime && theRobotPose.timeSinceLastValid < robotPoseLostTime) {
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
      if (theBallModel.timeSinceLastSeen < ballValidTime) {
        goto approachBall;
      }
      if (theBallModel.timeSinceLastSeen > ballLostTime && theRobotPose.timeSinceLastValid > robotPoseLostTime) {
        goto ballLostNotRelocalized;
      }
    }
    action {
      switch (theRobotBehaviorConfig.role) {
      case BehaviorStatus::striker:
        StrikerSearch();
        break;
      case BehaviorStatus::defender:
        DefenderSearch();
        break;
      case BehaviorStatus::supporter:
        SupporterSearch();
        break;
      case BehaviorStatus::keeper:
        KeeperSearch();
        break;
      case BehaviorStatus::goToBallAndKick:
      case BehaviorStatus::penaltyKeeper:
      case BehaviorStatus::penaltyStriker:
      case BehaviorStatus::striker2v2:
      case BehaviorStatus::defender2v2:
      case BehaviorStatus::dummy:
      case BehaviorStatus::testRole:
        DefenderSearch();
        break;
      }
    }
  }

  state(approachBall) {
    transition {
      if (theBallModel.timeSinceLastSeen > ballLostTime) {
        goto ballLostNotRelocalized;
      } else if (theBallModel.estimate.position.abs() < norminalDistance + 20) {
        if (theRobotPose.timeSinceLastValid <= robotPoseValidTime) {
          goto success;
        } else if (theRobotPose.timeSinceLastValid >= robotPoseLostTime) {
          goto hasBallNotReLocalized;
        }

      } else if (!libCodeRelease.hasBallLock(false)) {
        goto success;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      libCodeRelease.hasBallLock(true);
      WalkToTarget(Pose2D(1.f, 1.f, 1.f), Pose2D(relBallAngle, targetPosition));
    }
  }

  state(hasBallNotReLocalized) {
    transition {
      if (theBallModel.timeSinceLastSeen > ballLostTime && theRobotPose.timeSinceLastValid < robotPoseValidTime) {
        goto ballLostRelocalized;
      } else if (theBallModel.timeSinceLastSeen > ballLostTime && theRobotPose.timeSinceLastValid > robotPoseLostTime) {
        goto ballLostNotRelocalized;
      } else if (theRobotPose.timeSinceLastValid < robotPoseLostTime &&
                 theBallModel.estimate.position.abs() <= targetDistanceRange.max) {
        goto success;
      } else if (theBallModel.estimate.position.abs() > targetDistanceRange.max) {
        goto approachBall;
      } else if (action_aborted) {
        goto hasBallNotReLocalized;
      }
    }
    action { RelocateWhileProtectBall(); }
  }

  target_state(success) {
    action { theHeadControlMode = HeadControl::lookAtBall; }
  }
}
