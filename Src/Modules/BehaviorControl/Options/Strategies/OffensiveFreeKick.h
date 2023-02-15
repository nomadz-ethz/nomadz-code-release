/**
 * @file OffensiveFreeKick.h
 *
 * Striker/Supporter Behavior During a Free Kick
 * If player is from the kicking team and:
 * 1. It is a goal free kick -> go to your home location and do a local search
 * 2. If it is a pushing free kick find the ball (local search) and attack.
 * If player is from the non-kicking team:
 * local search and if ball is found move away from the ball.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(OffensiveFreeKick,
       const Pose2D& searchPointGoalFreeKick,
       const Pose2D& searchPointCornerFreeKick,
       const Pose2D& searchPointKickIn) {
  const Vector2<> gloOpponentGoal(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);

  Vector2<> gloBall = relToGlo(theBallModel.estimate.position);
  bool wantsBall = theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam &&
                   gloBall.x > theFieldDimensions.xPosOwnGroundline * 0.2 && libCodeRelease.timeSinceBallWasSeen() < 2500;

  Vector2<> cornerLocation = Vector2<>(theFieldDimensions.xPosOpponentGroundline,
                                       (searchPointCornerFreeKick.translation.y < 0 ? theFieldDimensions.yPosRightSideline
                                                                                    : theFieldDimensions.yPosLeftSideline));

  initial_state(alert) {
    transition {
      if (theGameInfo.setPlay == SET_PLAY_GOAL_KICK ||
          (theGameInfo.setPlay == SET_PLAY_CORNER_KICK && theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam) ||
          theGameInfo.setPlay == SET_PLAY_KICK_IN) {
        goto walkToSearchPoint;
      } else {
        goto localSearch;
      }
      action { Stand(); }
    }
  }

  state(walkToSearchPoint) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() < 500.f && theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam) {
        goto kickBall;
      } else if (libCodeRelease.timeSinceBallWasSeen() < 500.f && theOwnTeamInfo.teamNumber != theGameInfo.kickingTeam) {
        goto moveAwayFromBall;
      }

      if ((theGameInfo.setPlay == SET_PLAY_GOAL_KICK &&
           (theRobotPose.translation - searchPointGoalFreeKick.translation).abs() < 200.f) ||
          (theGameInfo.setPlay == SET_PLAY_KICK_IN &&
           (theRobotPose.translation - searchPointKickIn.translation).abs() < 200.f)) {
        goto localSearch;
      }
    }
    action {
      if (theGameInfo.setPlay == SET_PLAY_GOAL_KICK) {
        theHeadControlMode = HeadControl::walkScan;
        TravelTo(searchPointGoalFreeKick);
      } else if (theGameInfo.setPlay == SET_PLAY_CORNER_KICK) {
        lookTargetAngle = angleToGlobalTarget(cornerLocation);
        theHeadControlMode = HeadControl::lookAtTarget;
        TravelTo(searchPointCornerFreeKick);
      } else if (theGameInfo.setPlay == SET_PLAY_KICK_IN) {
        theHeadControlMode = HeadControl::walkScan;
        TravelTo(searchPointKickIn);
      }
    }
  }

  state(localSearch) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() < 500.f && theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam) {
        goto kickBall;
      } else if (libCodeRelease.timeSinceBallWasSeen() < 500.f && theOwnTeamInfo.teamNumber != theGameInfo.kickingTeam) {
        goto moveAwayFromBall;
      }

      if (action_aborted) {
        goto alert;
      }
      if ((theGameInfo.setPlay == SET_PLAY_GOAL_KICK &&
           (theRobotPose.translation - searchPointGoalFreeKick.translation).abs() > 200.f) ||
          (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam && theGameInfo.setPlay == SET_PLAY_CORNER_KICK &&
           (theRobotPose.translation - searchPointCornerFreeKick.translation).abs() > 200.f) ||
          (theGameInfo.setPlay == SET_PLAY_KICK_IN &&
           (theRobotPose.translation - searchPointKickIn.translation).abs() > 200.f)) {
        goto walkToSearchPoint;
      }
    }
    action { ScanRotate(); }
  }

  state(kickBall) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() > 2000.f) {
        goto alert;
      }

      if (!libCodeRelease.hasBallLock(wantsBall)) {
        goto moveAwayFromBall;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      if (theGameInfo.setPlay == SET_PLAY_CORNER_KICK) {
        // do not shoot at opponent goal with corner kick
        KickTo(gloOpponentGoal + Vector2<>(-500, 0));
      } else {
        KickTo(gloOpponentGoal);
      }
    }
  }

  state(moveAwayFromBall) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() > 2000.f) {
        goto alert;
      }

      if (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam && libCodeRelease.hasBallLock(wantsBall)) {
        goto kickBall;
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
}
