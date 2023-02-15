/**
 * @file DefenderSearch.h
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(DefenderSearch) {
  //    const Vector2<> patrolPointFront = (theRobotInfo.number % 2 == 0) ? Vector2<>(-1500.f, 833.f) : Vector2<>(-1500.f,
  //    -833.f);
  const Vector2<> patrolPointBase = (theRobotInfo.number % 2 == 0) ? setPosDefenderEven : setPosDefenderOdd;
  Vector2<> gloBall = theCombinedWorldModel.ballStateOthers.position;

  // Assumes Defenders are numbers 2 and 3
  const int otherDefender = (theRobotInfo.number == 2) ? 3 : 2;

  common_transition {
    if (libCodeRelease.timeSinceBallWasSeen() < ballValidTime) {
      goto success;
    }
    if (theRobotPose.timeSinceLastValid > 5000.f) {
      goto lost;
    }

    if (theCombinedWorldModel.timeSinceBallLastSeenOthers < 5000.f) {
      if ((!ballInKeeperClearArea() || !theTeamMateData.isActive[1]) &&
          (!theTeamMateData.isActive[otherDefender] ||
           (theRobotPose.translation - gloBall).abs() <
             (theTeamMateData.robotPoses[otherDefender].translation - gloBall).abs())) {
        goto guidedSearch;
      } else {
        goto guidedLook;
      }
    }
  }

  initial_state(localSearch) {
    transition {
      if (state_time > 7500 || std::abs(theRobotPose.translation.y) > theFieldDimensions.yPosLeftSideline - 600.f ||
          std::abs(theRobotPose.translation.x) > theFieldDimensions.xPosOpponentGroundline - 600.f) {
        goto walkToBase;
      }
      if (action_aborted) {
        goto walkToBase;
      }
    }
    action { ScanRotate(); }
  }

  state(walkToBase) {
    transition {
      if (std::abs(theRobotPose.translation.x - patrolPointBase.x) < 200.f &&
          std::abs(theRobotPose.translation.y - patrolPointBase.y) < 100.f &&
          std::abs(theRobotPose.rotation) < fromDegrees(12.f)) {
        goto baseSearch;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      TravelTo(Pose2D(0, patrolPointBase));
    }
  }

  state(baseSearch) {
    transition {
      if (std::abs(theRobotPose.translation.x - patrolPointBase.x) > 200.f ||
          std::abs(theRobotPose.translation.y - patrolPointBase.y) > 100.f ||
          std::abs(theRobotPose.rotation) > fromDegrees(12.f)) {
        goto walkToBase;
      }
      if (state_time > 2000) {
        goto baseSearchRight;
      }
    }
    action {
      theHeadControlMode = HeadControl::scanLeftRightKeeper;
      Stand();
    }
  }

  state(baseSearchRight) {
    transition {
      if (std::abs(theRobotPose.translation.x - patrolPointBase.x) > 200.f ||
          std::abs(theRobotPose.translation.y - patrolPointBase.y) > 100.f) {
        goto walkToBase;
      }
      if (state_time > 2000) {
        goto baseSearchLeft;
      }
    }
    action {
      theHeadControlMode = HeadControl::scanLeftRightKeeper;
      WalkToTarget(Pose2D(1.f, 0.f, 0.f), Pose2D(-30, Vector2<>(0, 0)));
    }
  }
  state(baseSearchLeft) {
    transition {
      if (std::abs(theRobotPose.translation.x - patrolPointBase.x) > 200.f ||
          std::abs(theRobotPose.translation.y - patrolPointBase.y) > 100.f) {
        goto walkToBase;
      }
      if (state_time > 2000) {
        goto baseSearch;
      }
    }
    action {
      theHeadControlMode = HeadControl::scanLeftRightKeeper;
      WalkToTarget(Pose2D(1.f, 0.f, 0.f), Pose2D(60, Vector2<>(0, 0)));
    }
  }

  state(guidedSearch) {
    transition {
      if (state_time > 20000.f || theCombinedWorldModel.timeSinceBallLastSeenOthers > 5000.f) {
        goto localSearch;
      }
      if (ballInKeeperClearArea()) {
        goto walkToBase;
      }
      if ((theTeamMateData.isActive[otherDefender] &&
           (theRobotPose.translation - gloBall).abs() >
             (theTeamMateData.robotPoses[otherDefender].translation - gloBall).abs())) {
        goto guidedLook;
      }
    }
    action {
      if (theCombinedWorldModel.ballStateOthers.position.x >= 0) {
        float interceptX = -2000.f;
        float interceptY = (theFieldDimensions.xPosOwnGroundline - interceptX) /
                           (theFieldDimensions.xPosOwnGroundline - gloBall.x) * gloBall.y;
        theHeadControlMode = HeadControl::lookAtBall;
        TravelTo(Pose2D((theCombinedWorldModel.ballStateOthers.position - theRobotPose.translation).angle(),
                        {interceptX, interceptY}));
      } else {
        theHeadControlMode = HeadControl::lookAtBall;
        TravelTo(Pose2D((theCombinedWorldModel.ballStateOthers.position - theRobotPose.translation).angle(),
                        theCombinedWorldModel.ballStateOthers.position));
      }
    }
  }

  state(guidedLook) {
    transition {
      if (theCombinedWorldModel.timeSinceBallLastSeenOthers > 5000.f || state_time > 20000) {
        goto walkToBase;
      }
      if (!theTeamMateData.isActive[otherDefender] ||
          (theRobotPose.translation - gloBall).abs() <
            (theTeamMateData.robotPoses[otherDefender].translation - gloBall).abs()) {
        goto guidedSearch;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToTarget(Pose2D(0.25f, 0.5f, 0.5f),
                   Pose2D(angleToGlobalTarget(theCombinedWorldModel.ballStateOthers.position), gloToRel(patrolPointBase)));
    }
  }

  target_state(success) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() > 2000) {
        goto localSearch;
      }
    }

    // Handled by caller option
  }

  aborted_state(lost) {
    // Handled by caller option
  }
}
