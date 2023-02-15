/**
 * @file SupporterSearch.h
 *
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iostream>
#include <vector>

Pose2D targetOne_Supporter = {0.f, setPosSupporterTargetOne};
Pose2D targetTwo_Supporter = {0.f, setPosSupporterTargetTwo};

option(SupporterSearch) {

  static bool atTargetOne_Supporter;
  static bool atTargetTwo_Supporter;

  common_transition {
    if (theRobotPose.timeSinceLastValid > 5000.f) {
      goto lost;
    }
    if (libCodeRelease.timeSinceBallWasSeen() < 300.f) {
      goto success;
    }
    if (theCombinedWorldModel.timeSinceBallLastSeenOthers < 5000.f) {
      if (theCombinedWorldModel.ballStateOthers.position.x > -1000.f ||
          (!ballInKeeperClearArea() && !libCodeRelease.isAnyTeamMateActive({2, 3}))) {
        goto guidedSearch;
      } else {
        goto guidedLook;
      }
    }
  }

  initial_state(localSearch) {

    transition {
      if (state_time > 10000 || action_aborted ||
          ((theRobotPose.translation.x < 0.f ||
            std::abs(theRobotPose.translation.x) > theFieldDimensions.xPosOpponentGroundline - 500.f ||
            std::abs(theRobotPose.translation.y) > theFieldDimensions.yPosLeftSideline - 500.f) &&
           state_time > 2500)) {
        if (atTargetOne_Supporter && !atTargetTwo_Supporter) {
          goto walkToTargetTwo;
        } else if (!atTargetOne_Supporter && atTargetTwo_Supporter) {
          goto walkToTargetOne;
        } else {
          if (theRobotPose.translation.y < 0) {
            goto walkToTargetOne;
          } else {
            goto walkToTargetTwo;
          }
        }
      }
    }

    action { ScanRotate(); }
  }

  state(walkToTargetOne) {
    transition {
      if (insideTargetArea_Supporter(targetOne_Supporter.translation)) {
        atTargetOne_Supporter = true;
        atTargetTwo_Supporter = false;
        goto waitToAttack;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      TravelTo(targetOne_Supporter);
    }
  }

  state(walkToTargetTwo) {
    transition {
      if (insideTargetArea_Supporter(targetTwo_Supporter.translation)) {
        atTargetTwo_Supporter = true;
        atTargetOne_Supporter = false;
        goto waitToAttack;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      TravelTo(targetTwo_Supporter);
    }
  }

  state(walkToWorldModelBall) {
    transition {
      if (theCombinedWorldModel.timeSinceBallLastSeenOthers > 5000.f ||
          theCombinedWorldModel.ballStateOthers.position.x > -1000) {
        goto localSearch;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      TravelTo({0.f, theCombinedWorldModel.ballStateOthers.position});
    }
  }

  target_state(success) {}

  aborted_state(lost) {}

  state(waitToAttack) {
    transition {
      if (state_time > 5000) {
        goto localSearch;
      }
    }
    action { ScanRotate(); }
  }

  state(guidedSearch) {
    transition {
      if (theCombinedWorldModel.timeSinceBallLastSeenOthers > 5000.f || state_time > 20000 || ballInKeeperClearArea()) {
        goto localSearch;
      }
      if (theCombinedWorldModel.ballStateOthers.position.x <= -1000 && libCodeRelease.isAnyTeamMateActive({2, 3})) {
        goto guidedLook;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      Vector2<> walkToBallTargetGlobal = theCombinedWorldModel.ballStateOthers.position;
      TravelTo(Pose2D(0, walkToBallTargetGlobal));
    }
  }

  state(guidedLook) {
    transition {
      if (theCombinedWorldModel.timeSinceBallLastSeenOthers > 5000.f || state_time > 20000) {
        goto localSearch;
      }
      if (theCombinedWorldModel.ballStateOthers.position.x > -1000 || !libCodeRelease.isAnyTeamMateActive({2, 3})) {
        goto guidedSearch;
      }
    }
    action {
      float interceptX = setPosSupporterTargetOne.x;
      Vector2<> gloBall = theCombinedWorldModel.ballStateOthers.position;
      float interceptY = (theFieldDimensions.xPosOpponentGroundline - interceptX) /
                         (theFieldDimensions.xPosOpponentGroundline - gloBall.x) * gloBall.y;
      theHeadControlMode = HeadControl::lookAtBall;
      TravelTo(Pose2D(pi, {interceptX, interceptY}));
    }
  }
}

/*Function to check if the robot is close to an assigned area passed via argument*/
bool insideTargetArea_Supporter(Vector2<> targetArea) {
  float x = theRobotPose.translation.x;
  float y = theRobotPose.translation.y;

  if ((x > (targetArea.x - 200)) && (x < (targetArea.x + 200.f)) && (y > (targetArea.y - 200)) &&
      (y < (targetArea.y + 200.f))) {
    return true;
  } else {
    return false;
  }
}
