/**
 * @file Defender.h
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(Defender) {
  Vector2<> posGoalDefence = {(theFieldDimensions.xPosOwnGroundline + theFieldDimensions.xPosOwnPenaltyArea) / 2,
                              theFieldDimensions.yPosCenterGoal};

  // go to ball if ball in our half, we are closest to ball and our estimate corresponds to global one
  float hardXLimit = theFieldDimensions.centerCircleRadius;
  float xLimit = 0.f;
  if (!theTeamMateData.isActive[4] || !theTeamMateData.isActive[5]) {
    hardXLimit = theFieldDimensions.xPosOpponentGroundline * 0.5;
  }

  const float activeExtraBallScore = -300.f;

  bool shouldDefendGoal =
    !theTeamMateData.isActive[1] && (theRobotInfo.number == 2 || (theRobotInfo.number == 3 && !theTeamMateData.isActive[2]));
  const float clearAreatolX = 400.f;
  const float clearAreatolY = 250.f;
  const bool ballInKeeperClearArea =
    relToGlo(theBallModel.estimate.position).x >= theFieldDimensions.xPosOwnGroundline &&
    relToGlo(theBallModel.estimate.position).x <= theFieldDimensions.xPosOwnGoalBox + clearAreatolX &&
    std::abs(relToGlo(theBallModel.estimate.position).y) <= theFieldDimensions.yPosLeftGoalBox + clearAreatolY;
  bool wantsBall = relToGlo(theBallModel.estimate.position).x < xLimit && libCodeRelease.timeSinceBallWasSeen() < 1500.f;

  common_transition {
    if (theGameInfo.setPlay == SET_PLAY_GOAL_KICK || theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK ||
        theGameInfo.setPlay == SET_PLAY_CORNER_KICK || theGameInfo.setPlay == SET_PLAY_KICK_IN) {
      goto freeKick;
    }
    if (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK) {
      goto penaltyKick;
    }
    if (theRobotPose.timeSinceLastValid > 5000.f) {
      goto relocate;
    }
    if (!shouldDefendGoal && libWorldModel.timeSinceBallLastSeen() > 7500.f) {
      goto search;
    }
    // NOTE: ball score will be set lower if we are currently playing
    if (!shouldDefendGoal && theBallModel.timeSinceLastSeen < 1500.f && libCodeRelease.hasBallLock(wantsBall, 0)) {
      goto play;
    }
  }

  state(relocate) {
    transition {
      if (action_done) {
        goto search;
      }
    }
    action { Relocate(); }
  }

  initial_state(search) {
    transition {
      if (action_aborted) {
        goto relocate;
      }
      if (shouldDefendGoal) {
        goto defendGoal;
      }
      if (libWorldModel.timeSinceBallLastSeen() < 2500.f || action_done) {
        goto defendPosition;
      }
    }
    action { DefenderSearch(); }
  }

  state(defendPosition) {
    transition {
      if (shouldDefendGoal) {
        goto defendGoal;
      }
    }
    action { DefenderPosition(); }
  }

  state(play) {
    transition {
      if (!libCodeRelease.hasBallLock(wantsBall, activeExtraBallScore) || theBallModel.timeSinceLastSeen > 1500.f) {
        if (libWorldModel.timeSinceBallLastSeen() < 2500.f) {
          goto defendPosition;
        } else {
          goto search;
        }
      }
    }
    action { DefenderPlay(hardXLimit); }
  }

  state(freeKick) {
    transition {
      if (theGameInfo.setPlay == SET_PLAY_NONE) {
        goto search;
      }
    }
    action { DefenderFreeKick(); }
  }

  state(penaltyKick) {
    transition {
      if (theGameInfo.setPlay == SET_PLAY_NONE) {
        goto search;
      }
    }
    action { StandHigh(); }
  }

  state(defendGoal) {
    transition {
      if (theTeamMateData.isActive[1]) {
        if (theBallModel.timeSinceLastSeen < 1500.f && libCodeRelease.hasBallLock(wantsBall, 0)) {
          goto play;
        } else {
          goto search;
        }
      }
      if ((theRobotPose.translation - posGoalDefence).abs() < 100 && std::abs(theRobotPose.rotation) < 0.1 &&
          state_time > 5000) {
        goto isInsideGoal;
      }
    }
    action {

      wantsBall = ballInKeeperClearArea && libCodeRelease.timeSinceBallWasSeen() < 1500.f;

      if (theBallModel.timeSinceLastSeen < ballValidTime && libCodeRelease.hasBallLock(wantsBall, 0)) {
        const Vector2<> opponentGoal =
          Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);
        theHeadControlMode = HeadControl::lookAtBall;
        KickTo(opponentGoal, Pose2D(fromDegrees(60.f), 0.f, 0.f));
      } else if (theRobotPose.translation.x < theFieldDimensions.xPosOwnPenaltyArea &&
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
      wantsBall = ballInKeeperClearArea && libCodeRelease.timeSinceBallWasSeen() < 1500.f;
      if (theTeamMateData.isActive[1]) {
        if (theBallModel.timeSinceLastSeen < ballValidTime && libCodeRelease.hasBallLock(wantsBall, 0)) {
          goto play;
        } else {
          goto search;
        }
      }
      if ((theRobotPose.translation - posGoalDefence).abs() > 100 || action_done) {
        goto defendGoal;
      }
    }
    action {
      theHeadControlMode = HeadControl::scanLeftRightKeeper;
      if (theBallModel.timeSinceLastSeen < ballValidTime && libCodeRelease.hasBallLock(wantsBall, 0)) {
        const Vector2<> opponentGoal =
          Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);
        theHeadControlMode = HeadControl::lookAtBall;
        KickTo(opponentGoal, Pose2D(fromDegrees(60.f), 0.f, 0.f));
      } else if (theCombinedWorldModel.timeSinceBallLastSeen < ballValidTime &&
                 theCombinedWorldModel.ballState.position.x >= 0) {
        StandHigh();
      } else if (theCombinedWorldModel.timeSinceBallLastSeen < ballValidTime) {
        Angle relAngleToBall =
          atan2(gloToRel(theCombinedWorldModel.ballState.position).y, gloToRel(theCombinedWorldModel.ballState.position).x);
        WalkToTarget(Pose2D(0.5f, 0.f, 0.f), Pose2D(relAngleToBall, 0));
      } else {
        KeeperSearch();
      }
    }
  }
}
