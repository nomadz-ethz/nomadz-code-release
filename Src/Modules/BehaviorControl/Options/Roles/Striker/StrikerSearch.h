/**
 * @file StrikerSearch.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iostream>
#include <vector>

Pose2D targetOne_Striker = {1.57f, setPosStrikerTargetOne}; // hardcoded to be in opponent side for now
Pose2D targetTwo_Striker = {-1.57f, setPosStrikerTargetTwo};

option(StrikerSearch) {

  static bool atTargetOne_Striker;
  static bool atTargetTwo_Striker;

  auto kickEstimate = libWorldModel.kickEstimate(5000.f);
  if (kickEstimate.x > theFieldDimensions.xPosOpponentGoalBox) {
    kickEstimate.x = theFieldDimensions.xPosOpponentGoalBox;
  }

  common_transition {
    if (theRobotPose.timeSinceLastValid > 5000.f) {
      atTargetOne_Striker = false;
      atTargetTwo_Striker = false;
      goto lost;
    }
    if (libCodeRelease.timeSinceBallWasSeen() < 300.f) {
      atTargetOne_Striker = false;
      atTargetTwo_Striker = false;
      goto success;
    }
    if (theCombinedWorldModel.timeSinceBallLastSeenOthers < 5000.f) {
      if (theCombinedWorldModel.ballStateOthers.position.x > 0.f ||
          (!ballInKeeperClearArea() && !libCodeRelease.isAnyTeamMateActive({2, 3, 4}))) {
        goto guidedSearch;
      } else {
        goto guidedLook;
      }
    }
    if ((!isnan(kickEstimate.x) && !isnan(kickEstimate.y)) && kickEstimate.x > 0 &&
        std::fabs(kickEstimate.y) < theFieldDimensions.xPosOpponentGoalBox) {
      goto checkKickEstimate;
    }
  }

  initial_state(localSearch) {

    transition {
      if (state_time > 10000 || action_aborted ||
          ((theRobotPose.translation.x < 0.f ||
            std::abs(theRobotPose.translation.x) > theFieldDimensions.xPosOpponentGroundline - 500.f ||
            std::abs(theRobotPose.translation.y) > theFieldDimensions.yPosLeftSideline - 500.f) &&
           state_time > 2500)) {
        if (atTargetOne_Striker && !atTargetTwo_Striker) {
          goto walkToTargetTwo;
        } else if (!atTargetOne_Striker && atTargetTwo_Striker) {
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
      if (insideTargetArea_Striker(targetOne_Striker)) {
        atTargetOne_Striker = true;
        atTargetTwo_Striker = false;
        goto waitToAttack;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      TravelTo(targetOne_Striker);
    }
  }

  state(walkToTargetTwo) {
    transition {
      if (insideTargetArea_Striker(targetTwo_Striker)) {
        atTargetTwo_Striker = true;
        atTargetOne_Striker = false;
        goto waitToAttack;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      TravelTo(targetTwo_Striker);
    }
  }

  state(checkKickEstimate) {
    transition {
      if (state_time > 5000 || isnan(kickEstimate.x) || isnan(kickEstimate.y)) {
        goto localSearch;
      }
    }
    action {
      if (!isnan(kickEstimate.x) || isnan(kickEstimate.y)) {
        lookTargetAngle = angleToGlobalTarget(kickEstimate);
        theHeadControlMode = HeadControl::lookAtTarget;
        TravelTo(kickEstimate);
      }
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
      if (theCombinedWorldModel.ballStateOthers.position.x <= 0 && libCodeRelease.isAnyTeamMateActive({2, 3, 4})) {
        goto guidedLook;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      Vector2<> walkToBallTargetGlobal = theCombinedWorldModel.ballStateOthers.position;
      TravelTo(Pose2D(0, walkToBallTargetGlobal));
    }
  }

  state(guidedLook) {
    transition {
      if (theCombinedWorldModel.timeSinceBallLastSeenOthers > 5000.f || state_time > 20000) {
        goto localSearch;
      }
      if (theCombinedWorldModel.ballStateOthers.position.x > 0 || !libCodeRelease.isAnyTeamMateActive({2, 3, 4})) {
        goto guidedSearch;
      }
    }
    action {
      float interceptX = setPosStrikerTargetOne.x;
      Vector2<> gloBall = theCombinedWorldModel.ballStateOthers.position;
      float interceptY = (theFieldDimensions.xPosOpponentGroundline - interceptX) /
                         (theFieldDimensions.xPosOpponentGroundline - gloBall.x) * gloBall.y;
      theHeadControlMode = HeadControl::lookAtBall;
      TravelTo(Pose2D(pi, {interceptX, interceptY}));
    }
  }
}

/*Function to check if the robot is close to an assigned area passed via argument*/
bool insideTargetArea_Striker(Pose2D targetArea) {
  float x = theRobotPose.translation.x;
  float y = theRobotPose.translation.y;
  float rot = theRobotPose.rotation;
  if (std::fabs(x - targetArea.translation.x) < 200.f && std::fabs(y - targetArea.translation.y) < 200.f &&
      std::fabs(rot - targetArea.rotation) < 0.3) {
    return true;
  } else {
    return false;
  }
}
