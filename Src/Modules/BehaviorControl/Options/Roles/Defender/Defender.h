/**
 * @file Defender.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(Defender) {
  Vector2<> posGoalDefence = {(theFieldDimensions.xPosOwnGroundline + theFieldDimensions.xPosOwnPenaltyArea) / 2,
                              theFieldDimensions.yPosCenterGoal};

  // go to ball if ball in our half, we are closest to ball and our estimate corresponds to global one
  // float hardXLimit = theFieldDimensions.centerCircleRadius;
  // float xLimit = 0.f;
  // if (!theTeamMateData.isActive[4] || !theTeamMateData.isActive[5]) {
  //   hardXLimit = theFieldDimensions.xPosOpponentGroundline * 0.5;
  // }

  const float activeExtraBallScore = -300.f;

  bool shouldDefendGoal = !theTeamMateData.isActive[1] && (libCodeRelease.getOtherActiveDefenders(theRobotInfo.number) == 0);
  const float clearAreatolX = 400.f;
  const float clearAreatolY = 250.f;
  const bool ballInKeeperClearArea =
    relToGlo(theBallModel.estimate.position).x >= theFieldDimensions.xPosOwnGroundline &&
    relToGlo(theBallModel.estimate.position).x <= theFieldDimensions.xPosOwnGoalBox + clearAreatolX &&
    std::abs(relToGlo(theBallModel.estimate.position).y) <= theFieldDimensions.yPosLeftGoalBox + clearAreatolY;

  common_transition {
    // NOTE: ball score will be set lower if we are currently playing
  }

  initial_state(defendPosition) {
    transition {
      if (shouldDefendGoal) {
        goto defendGoal;
      }
    }
    action { DefenderPosition(); }
  }

  state(defendGoal) {
    transition {
      if (theTeamMateData.isActive[1]) {
        goto defendPosition;
      }
      if ((theRobotPose.translation - posGoalDefence).abs() < 100 && std::abs(theRobotPose.rotation) < 0.1 &&
          state_time > 5000) {
        goto isInsideGoal;
      }
    }
    action {
      if (theRobotPose.translation.x < theFieldDimensions.xPosOwnPenaltyArea &&
          std::abs(theRobotPose.translation.y) < theFieldDimensions.yPosLeftPenaltyArea) {
        theHeadControlMode = HeadControl::walkScan;
        WalkToTarget(Pose2D(1.f, 1.f, 1.f), Pose2D(-theRobotPose.rotation, gloToRel(posGoalDefence)));
      } else {
        theHeadControlMode = HeadControl::walkScan;
        TravelTo(Pose2D(0, posGoalDefence));
      }
    }
  }

  state(isInsideGoal) {
    transition {
      if (theTeamMateData.isActive[1]) {
        goto defendPosition;
      }
      if ((theRobotPose.translation - posGoalDefence).abs() > 100 || action_done) {
        goto defendGoal;
      }
    }
    action {
      theHeadControlMode = HeadControl::scanLeftRight;
      if (theCombinedWorldModel.timeSinceBallLastSeen < 300 && theCombinedWorldModel.ballState.position.x >= 0) {
        StandHigh();
      } else if (theCombinedWorldModel.timeSinceBallLastSeen < 300) {
        Angle relAngleToBall =
          atan2(gloToRel(theCombinedWorldModel.ballState.position).y, gloToRel(theCombinedWorldModel.ballState.position).x);
        WalkToTarget(Pose2D(0.5f, 0.f, 0.f), Pose2D(relAngleToBall, 0));
      } else {
        KeeperSearch();
      }
    }
  }
}
