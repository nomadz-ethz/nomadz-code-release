/**
 * @file OffensivePlay2v2.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iostream>
#include <vector>

option(OffensivePlay2v2) {

  const Pose2D midfield_recovery = {3.14, {static_cast<float>(theFieldDimensions.xPosOpponentGroundline * 0.2), 0}};

  const Vector2<> gloOpponentGoal(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);
  Vector2<> gloBall = libWorldModel.ballPosition(2000.f);

  bool ballLocationInRange = gloBall.x > theFieldDimensions.xPosOwnGroundline * 0.2;
  bool wantsBall = ballLocationInRange && libCodeRelease.timeSinceBallWasSeen() < 2500;

  initial_state(lookForBall) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() > 5000.f) {
        goto failure;
      } else if ((gloBall - gloOpponentGoal).abs() > theFieldDimensions.xPosOpponentGroundline * 1.2) { // 3000
        goto moveBack;
      } else if (libCodeRelease.timeSinceBallWasSeen() < 1000.f) {
        goto kick;
      }
    }
    action {
      Pose2D lookingBack = {3.14, theRobotPose.translation};
      TravelTo(lookingBack);
      ScanLeftRight();
    }
  }

  state(moveBack) {
    transition {
      if ((theRobotPose.translation).abs() < 1000) {
        goto wait;
      } else if ((libCodeRelease.timeSinceBallWasSeen() < 1000.f) &&
                 ((gloBall - gloOpponentGoal).abs() < theFieldDimensions.xPosOpponentGroundline)) {
        goto kick;
      }
    }
    action { TravelTo(midfield_recovery); }
  }

  state(wait) {
    transition {
      if (state_time > 1000.f) {
        goto lookForBall;
      }
    }
    action { Stand(); }
  }

  state(kick) {
    transition {
      if ((gloBall - gloOpponentGoal).abs() >
          (theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyMark) * 1.7) { // 1250
        goto dribble;
      } else if (libCodeRelease.nearestPlayerInSight(0.8, 500) < 500 &&
                 (gloBall - gloOpponentGoal).abs() >
                   (theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyMark)) { // 750
        goto dribble;
      } else if (libCodeRelease.timeSinceBallWasSeen() > 2000.f) {
        goto lookForBall;
      } else if (!libCodeRelease.hasBallLock(wantsBall, 0.f)) {
        goto lookForBall;
      }
    }
    action { ScoreGoal2v2(); }
  }

  state(dribble) {
    transition {
      if ((gloBall - gloOpponentGoal).abs() <
            (theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyMark) * 1.3 &&
          libCodeRelease.nearestPlayerInSight(0.8, 500) > 750) {
        goto kick;
      } else if (libCodeRelease.timeSinceBallWasSeen() > 2000.f) {
        goto lookForBall;
      } else if ((gloBall - gloOpponentGoal).abs() > theFieldDimensions.xPosOpponentGroundline * 1.2) { // 3000
        goto moveBack;
      } else if (!libCodeRelease.hasBallLock(wantsBall, 0.f)) {
        goto lookForBall;
      }
    }
    action { DribbleTo(gloOpponentGoal, fromDegrees(30.f)); }
  }

  aborted_state(failure) { ; }
}