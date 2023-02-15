/**
 * @file DefenderFreeKick
 *
 * Defender Behavior During a Free Kick
 * If player is from the kicking team and:
 * 1. It is a goal free kick -> go to your home location and do a local search
 *    (as the ball is likely placed there)
 * 2. If it is a pushing free kick find the ball (local search) and attack.
 * If player is from the non-kicking team:
 * local search and if ball is found move away from the ball.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(DefenderFreeKick) {
  const Vector2<> goalPost = (theRobotInfo.number % 2 == 0)
                               ? Vector2<>(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal)
                               : Vector2<>(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal);
  const Vector2<> gloOpponentGoal(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);
  const Vector2<> searchPointGoalFreeKick =
    (theRobotInfo.number % 2 == 0) ? posGoalFreeKickDefenderEven : posGoalFreeKickDefenderOdd;
  const float rotGoalFreeKick = (theRobotInfo.number % 2 == 0) ? -pi / 4 : pi / 4;
  const Vector2<> searchPointCornerFreeKick =
    (theRobotInfo.number % 2 == 0) ? posCornerFreeKickDefenderEven : posCornerFreeKickDefenderOdd;

  bool wantsBall = relToGlo(theBallModel.estimate.position).x < -500.f && libCodeRelease.timeSinceBallWasSeen() < 2500;

  initial_state(alert) {
    transition {
      if ((theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam && theGameInfo.setPlay == SET_PLAY_GOAL_KICK) ||
          (theOwnTeamInfo.teamNumber != theGameInfo.kickingTeam && theGameInfo.setPlay == SET_PLAY_CORNER_KICK)) {
        goto walkToSearchPoint;
      } else {
        goto localSearch;
      }
    }
    action { Stand(); }
  }

  state(walkToSearchPoint) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() < 300 && libCodeRelease.hasBallLock(wantsBall)) {
        goto kickBall;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      if (theGameInfo.setPlay == SET_PLAY_GOAL_KICK) {
        TravelTo(Pose2D(rotGoalFreeKick, searchPointGoalFreeKick));
      } else { // corner kick
        TravelTo(Pose2D(0.f, searchPointCornerFreeKick));
      }
    }
  }

  state(localSearch) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() < 300 && theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam) {
        goto kickBall;
      } else if (libCodeRelease.timeSinceBallWasSeen() < 300 && theOwnTeamInfo.teamNumber != theGameInfo.kickingTeam) {
        goto defendFreeKick;
      }

      if (action_aborted) {
        goto alert;
      }
    }
    action { ScanRotate(); }
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
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      KickTo(gloOpponentGoal);
    }
  }

  state(moveAwayFromBall) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() > 5000) {
        goto alert;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      if (theBallModel.estimate.position.abs() < 750.f) {
        WalkToTarget(Pose2D(1.f, 1.f, 0.4f), Pose2D(theBallModel.estimate.position.angle(), -200, 0));
      } else {
        Stand();
      }
    }
  }

  state(defendFreeKick) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() > 5000) {
        goto alert;
      }
      if ((goalPost - relToGlo(theBallModel.estimate.position)).abs() < 1000 ||
          (theBallModel.estimate.position.abs() <= 750.f &&
           relToGlo(theBallModel.estimate.position).x < theRobotPose.translation.x)) {
        goto moveAwayFromBall;
      }
    }
    action {
      float safePointRatio = ((relToGlo(theBallModel.estimate.position) - goalPost).abs() - 1000) /
                             (relToGlo(theBallModel.estimate.position) - goalPost).abs();
      Vector2<> safePoint(safePointRatio * (relToGlo(theBallModel.estimate.position) - goalPost).x,
                          safePointRatio * (relToGlo(theBallModel.estimate.position) - goalPost).y);
      Vector2<> defencePoint = Vector2<>(0.4 * safePoint.x, 0.4 * safePoint.y) + goalPost;
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToTarget(Pose2D(1.f, 1.f, 0.1f), Pose2D(theBallModel.estimate.position.angle(), gloToRel(defencePoint)));
    }
  }

  state(waitForBallUnlocking) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() > 5000) {
        goto alert;
      }

      if (libCodeRelease.hasBallLock(wantsBall)) {
        goto kickBall;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      if (theBallModel.estimate.position.abs() < 500.f) {
        WalkToTarget(Pose2D(1.f, 1.f, 0.4f), Pose2D(theBallModel.estimate.position.angle(), -200, 0));
      } else {
        Stand();
      }
    }
  }
}
