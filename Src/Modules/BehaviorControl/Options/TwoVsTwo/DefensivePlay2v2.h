/**
 * @file DefenderPlay.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(DefensivePlay2v2, const float xLimit) {
  const Vector2<> alertPointBase = {static_cast<float>(theFieldDimensions.xPosOwnGroundline * 0.7), 0};
  const float engageMinXDist = 1250.f;
  const float engageMinYDist = 500.f;

  const float engagePlayerMinDist = 100000.f; // 2000.f;
  const float passPlayerMinDist = 0.f;        // 1200.f;

  const float basePosTol = 200.f;
  const float baseAngleTol = fromDegrees(10.f);
  const float limitPass = -500;

  const Vector2<> gloBall = relToGlo(theBallModel.estimate.position);

  const Vector2<> gloOpponentGoal(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);

  // Is there at least one fully active offensive player far away
  // (far away = in opponent half TODO review "far" condition) ?
  const bool activeStrikerFarAway = true;

  // Is there at least one other player detected within [radius] of me in line of sight?
  float nearestPlayerInSight = libCodeRelease.nearestPlayerInSight(0.8, 500);

  initial_state(walkToBase) {
    transition {
      if (gloBall.x < xLimit) {
        // Engage ball if it's in own half
        goto engage;
      } else if (((alertPointBase - theRobotPose.translation).abs() > basePosTol) &&
                 (std::abs(0 - theRobotPose.rotation) > 2.f * baseAngleTol)) {
        goto wait;
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
      if (gloBall.x > xLimit) {
        goto walkToBase;
      }
      if ((activeStrikerFarAway) && (gloBall.x >= limitPass)) {
        goto pass;
      }
      if ((std::fabs(theBallModel.estimate.position.x < engageMinXDist) ||
           std::fabs(theBallModel.estimate.position.y) < engageMinYDist)) {
        goto chase;
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
      if ((std::fabs(theBallModel.estimate.position.x >= engageMinXDist) ||
           std::fabs(theBallModel.estimate.position.y) >= engageMinYDist) &&
          nearestPlayerInSight < engagePlayerMinDist) {
        goto engage;
      }
      if ((nearestPlayerInSight > passPlayerMinDist && activeStrikerFarAway) && gloBall.x >= limitPass) {
        goto pass;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      if (nearestPlayerInSight < 800.f) {
        // Possible danger! Large tolerance
        // TODO Own players can also cause this
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
      if (gloBall.x < limitPass) {
        goto engage;
      }
      if ((std::fabs(theBallModel.estimate.position.x >= engageMinXDist) ||
           std::fabs(theBallModel.estimate.position.y) >= engageMinYDist) &&
          nearestPlayerInSight < engagePlayerMinDist) {
        goto engage;
      }
      if ((nearestPlayerInSight <= 800.f || !activeStrikerFarAway) && (gloBall.x < 0)) {
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
        KickTo2v2(furthestStriker);
      } else {
        KickTo2v2(gloOpponentGoal);
      }
    }
  }

  state(wait) {
    transition {
      if (gloBall.x < xLimit) {
        goto engage;
      }
    }
    action { Stand(); }
  }
}
