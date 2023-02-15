/**
 * @file OneVsOneWait.h
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iostream>
#include <vector>

option(OneVsOneWait) {

  const Pose2D basePose =
    Pose2D(0.f, (theFieldDimensions.xPosOwnGroundline + theFieldDimensions.xPosOwnPenaltyArea) / 2.0, 0.0f);
  const float basePosTol = 100.f;
  const float baseAngleTol = fromDegrees(10.f);

  const Pose2D lookOnePose = Pose2D(0.f, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosLeftGoal);
  const Pose2D lookTwoPose = Pose2D(0.f, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosRightGoal);

  initial_state(gotoBase) {
    transition {
      if (((basePose.translation - theRobotPose.translation).abs() < basePosTol) &&
          (std::abs(basePose.rotation - theRobotPose.rotation) < baseAngleTol)) {
        goto waitBase;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      TravelTo(basePose);
    }
  }

  state(waitBase) {
    transition {
      if (((basePose.translation - theRobotPose.translation).abs() > 2.f * basePosTol) ||
          (std::abs(basePose.rotation - theRobotPose.rotation) > 2.f * baseAngleTol)) {
        goto gotoBase;
      }

      // Try searching something after 10 seconds at random spot
      if (state_time > 10000.0f) {
        if (rand() % 2) {
          goto lookPostOne;
        } else {
          goto lookPostTwo;
        }
      }
    }
    action {
      theHeadControlMode = HeadControl::scanLeftRight;
      StandHigh();
    }
  }

  state(lookPostOne) {
    transition {
      if (state_time > 10000.f) {
        goto gotoBase;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      TravelTo(lookOnePose);
    }
  }

  state(lookPostTwo) {
    transition {
      if (state_time > 10000.f) {
        goto gotoBase;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      TravelTo(lookTwoPose);
    }
  }
}
