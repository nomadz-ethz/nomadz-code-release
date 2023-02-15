/**
 * @file KeeperFreeKick
 *
 * Keeper freekick. Keeper only uses this when none of the defenders is available to take
 * the goal free kick
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(KeeperFreeKick) {
  const Vector2<> gloOpponentGoal(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);

  // Keeper wants the ball only for goal free kick (this option is only executed if there are no defenders)
  bool wantsBall = (theGameInfo.setPlay == SET_PLAY_GOAL_KICK);

  initial_state(alert) {
    transition {
      if (theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK || theGameInfo.setPlay == SET_PLAY_GOAL_KICK) {
        goto localSearch;
      }
    }
    action { Stand(); }
  }

  state(localSearch) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() < 300 && theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam) {
        goto kickBall;
      }
    }
    action {
      Stand();
      theHeadControlMode = HeadControl::scanLeftRightKeeper;
    }
  }

  state(kickBall) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() > 5000) {
        goto alert;
      }

      if (!libCodeRelease.hasBallLock(wantsBall)) {
        goto waitForBallUnlocking;
      }
    }
    action { KickTo(gloOpponentGoal); }
  }

  state(waitForBallUnlocking) {
    transition {
      if (theBallModel.estimate.position.abs() >= 800.f || libCodeRelease.timeSinceBallWasSeen() > 5000) {
        goto alert;
      }

      if (libCodeRelease.hasBallLock(wantsBall)) {
        goto kickBall;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      if (theBallModel.estimate.position.abs() < 500.f) {
        WalkToTarget(Pose2D(1.f, 1.0f, 0.5f), Pose2D(theBallModel.estimate.position.angle(), -200, 0));
      } else {
        Stand();
      }
    }
  }
}
