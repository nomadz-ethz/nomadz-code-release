/**
 * @file ActiveRoleGenericBehavior.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

bool teamMateHasBall = false;

option(ActiveRoleGenericBehavior) {
  float fallBackDistance = freeKick ? 850.f : 500.f;
  Vector2<> relCombinedBall = gloToRel(libWorldModel.ballPosition());

  initial_state(engageBall) {
    transition {
      if (!shouldEngageBall) {
        goto approachBallObservationPoint;
      }
    }
    action { EngageBall(); }
  }

  state(approachBallObservationPoint) {
    transition {
      if (shouldEngageBall) {
        goto engageBall;
      }
      if (!thePersonalData.shouldApproachBall) {
        goto roleSpecificBehavior;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      if (relCombinedBall.abs() > fallBackDistance + 50 && relCombinedBall.abs() < fallBackDistance + 150) {
        Stand();
      } else {
        WalkToTarget(Pose2D(1.f, 0.8f, 0.f),
                     Pose2D(relCombinedBall.angle(), relCombinedBall.x - fallBackDistance - 100.f, 0.f));
      }
    }
  }

  state(roleSpecificBehavior) {
    transition {
      if (shouldEngageBall) {
        goto engageBall;
      }
      if (thePersonalData.shouldApproachBall) {
        goto approachBallObservationPoint;
      }
    }
    action {
      switch (theBehaviorStatus.role) {
      case BehaviorStatus::striker:
        Striker();
        break;
      case BehaviorStatus::supporter:
        Supporter();
        break;
      case BehaviorStatus::defender:
        Defender();
        break;
      case BehaviorStatus::keeper:
        Keeper();
        break;
      default:
        Striker();
        break;
      }
    }
  }
}