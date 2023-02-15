/**
 * @file DefenderPlay.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(DefenderPlay, const float xLimit) {
  const Vector2<> alertPointBase = (theRobotInfo.number % 2 == 0) ? setPosDefenderEven : setPosDefenderOdd;
  const float engageMinXDist = 1250.f;
  const float engageMinYDist = 500.f;

  const float engagePlayerMinDist = 2000.f;
  const float passPlayerMinDist = 1200.f;

  const Vector2<> gloBall = relToGlo(theBallModel.estimate.position);

  const Vector2<> gloOpponentGoal(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);
  const bool ballLocationFree =
    (!libCodeRelease.inGoalBox(gloBall, theFieldDimensions.ballRadius) || libCodeRelease.numTeamMatesInGoalBox() < 2);

  // Is there at least one fully active offensive player far away
  // (far away = in opponent half TODO review "far" condition) ?
  const bool activeStrikerFarAway = (theTeamMateData.isFullyActive[4] &&
                                     (theTeamMateData.robotPoses[4].translation.x - theRobotPose.translation.x) >= 1500.f) ||
                                    (theTeamMateData.isFullyActive[5] &&
                                     (theTeamMateData.robotPoses[5].translation.x - theRobotPose.translation.x) >= 1500.f);

  // Is there at least one other player detected within [radius] of me in line of sight?
  float nearestPlayerInSight = libCodeRelease.nearestPlayerInSight(0.8, 500);

  common_transition {
    if (!ballLocationFree) {
      goto walkToBase;
    }
  }

  initial_state(walkToBase) {
    transition {
      if (gloBall.x < xLimit && ballLocationFree) {
        // Engage ball if it's in own half
        goto engage;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      TravelTo(Pose2D(0, alertPointBase));
    }
  }

  // Go towards the ball
  state(engage) {
    transition {
      if (gloBall.x > xLimit || !ballLocationFree) {
        goto walkToBase;
      }

      if ((std::fabs(theBallModel.estimate.position.x < engageMinXDist) &&
           std::fabs(theBallModel.estimate.position.y) < engageMinYDist) ||
          nearestPlayerInSight > engagePlayerMinDist) {
        if (activeStrikerFarAway) {
          goto pass;
        } else {
          goto chase;
        }
      }
    }
    action {
      Vector2<> intercept = Vector2<>(1000.f, 0);
      float distToLine = std::max(0.f, theFieldDimensions.yPosLeftSideline - std::fabs(theRobotPose.translation.y));
      intercept *= distToLine / theFieldDimensions.yPosLeftSideline;
      if (gloBall.x - intercept.x < theFieldDimensions.xPosOwnGroundline) {
        // go direct to the ball if you would go out of the field otherwise
        TravelTo(Pose2D(0, gloBall));
      } else {
        // move in front of the ball
        TravelTo(Pose2D(0, gloBall - intercept));
      }
    }
  }

  // If no active strikers far away OR other player detected near
  state(chase) {
    transition {
      if ((std::fabs(theBallModel.estimate.position.x > engageMinXDist) ||
           std::fabs(theBallModel.estimate.position.y) > engageMinYDist) &&
          nearestPlayerInSight < engagePlayerMinDist) {
        goto engage;
      }
      // We are running out of time with the ball and we still have ball lock, just kick it away
      if (gloBall.x > xLimit - 200.f) {
        goto forceKick;
      }
      if (nearestPlayerInSight > passPlayerMinDist && activeStrikerFarAway) {
        goto pass;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      if (libCodeRelease.nearestOpponentInSight(0.8, 500.f) < 800.f) {
        // Possible danger! Large tolerance
        DribbleTo(gloOpponentGoal, fromDegrees(60.f));
      } else {
        // Less danger: opponent more than 500 away. Be slightly more precise
        DribbleTo(gloOpponentGoal, fromDegrees(45.f));
      }
    }
  }

  // Only if at least one striker far away AND no other players detected near
  // (basically, have a target to pass to & have time to align for power-kick)
  state(pass) {
    transition {
      if (action_done) {
        goto walkToBase;
      }
      if ((std::fabs(theBallModel.estimate.position.x > engageMinXDist) ||
           std::fabs(theBallModel.estimate.position.y) > engageMinYDist) &&
          nearestPlayerInSight < engagePlayerMinDist) {
        goto engage;
      }
      // We are running out of time with the ball and we still have ball lock, just kick it away
      if (gloBall.x > xLimit - 200.f) {
        goto forceKick;
      }
      if (nearestPlayerInSight <= 800.f || !activeStrikerFarAway) {
        goto chase;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      Vector2<> gloPassTarget;
      Vector2<> furthestStriker;
      float furthestStrikerDistance = -1.f;
      if (theTeamMateData.isFullyActive[4] &&
          theTeamMateData.robotPoses[4].translation.x - theRobotPose.translation.x > furthestStrikerDistance) {
        furthestStrikerDistance = theTeamMateData.robotPoses[4].translation.x - theRobotPose.translation.x;
        furthestStriker = theTeamMateData.robotPoses[4].translation;
      }
      if (theTeamMateData.isFullyActive[5] &&
          theTeamMateData.robotPoses[5].translation.x - theRobotPose.translation.x > furthestStrikerDistance) {
        furthestStrikerDistance = theTeamMateData.robotPoses[5].translation.x - theRobotPose.translation.x;
        furthestStriker = theTeamMateData.robotPoses[5].translation;
      }

      if (furthestStrikerDistance > 0.f) {
        KickTo(furthestStriker);
      } else {
        KickTo(gloOpponentGoal);
      }
    }
  }

  state(forceKick) {
    transition {
      if (action_done) {
        goto walkToBase;
      }
      if (gloBall.x < xLimit - 500.f) {
        goto engage;
      }
    }
    action { KickTo(gloOpponentGoal); }
  }
}
