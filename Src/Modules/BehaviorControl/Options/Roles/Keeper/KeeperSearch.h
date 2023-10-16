/**
 * @file KeeperSearch.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(KeeperSearch) {
  Vector2<> posGoalDefence = {(theFieldDimensions.xPosOwnGroundline + theFieldDimensions.xPosOwnPenaltyArea) / 2,
                              theFieldDimensions.yPosCenterGoal};
  common_transition {}

  initial_state(walkToBase) {
    transition {
      if (std::abs(theRobotPose.translation.x - posGoalDefence.x) < 200.f &&
          std::abs(theRobotPose.translation.y - posGoalDefence.y) < 100.f &&
          std::abs(theRobotPose.rotation) < fromDegrees(12.f)) {
        goto baseSearch;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
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

  state(baseSearch) {
    transition {
      if (std::abs(theRobotPose.translation.x - posGoalDefence.x) > 200.f ||
          std::abs(theRobotPose.translation.y - posGoalDefence.y) > 100.f ||
          std::abs(theRobotPose.rotation) > fromDegrees(12.f)) {
        goto walkToBase;
      }
      if (state_time > 4000) {
        goto baseSearchRight;
      }
    }
    action {
      theHeadControlMode = HeadControl::scanLeftRight;
      Stand();
    }
  }

  state(baseSearchRight) {
    transition {
      if (std::abs(theRobotPose.translation.x - posGoalDefence.x) > 200.f ||
          std::abs(theRobotPose.translation.y - posGoalDefence.y) > 100.f) {
        goto walkToBase;
      }
      if (state_time > 2000) {
        goto stationaryRightCheck;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2D(1.f, 0.f, 0.f), Pose2D(-50, Vector2<>(0, 0)));
    }
  }

  state(stationaryRightCheck) {
    transition {
      if (state_time > 1000) {
        goto baseSearchLeft;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      Stand();
    }
  }

  state(baseSearchLeft) {
    transition {
      if (std::abs(theRobotPose.translation.x - posGoalDefence.x) > 200.f ||
          std::abs(theRobotPose.translation.y - posGoalDefence.y) > 100.f) {
        goto walkToBase;
      }
      if (state_time > 3500) {
        goto stationaryLeftCheck;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2D(1.f, 0.f, 0.f), Pose2D(-100, Vector2<>(0, 0)));
    }
  }

  state(stationaryLeftCheck) {
    transition {
      if (state_time > 1000) {
        goto success;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      Stand();
    }
  }

  target_state(success) {}

  aborted_state(lost) {}
}
