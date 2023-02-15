/**
 * @file Keeper.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

bool dived = false;
float lastSitDown = 0;

option(Keeper) {

  // x-axis offset of Keeper off goal line when in defending area
  const float offsetKeeperOffLine = (theFieldDimensions.xPosOwnGoalBox - theFieldDimensions.xPosOwnGroundline) / 3;
  // radius of danger area half circle
  const float dangerousDistance = (theFieldDimensions.xPosOwnPenaltyArea - theFieldDimensions.xPosOwnGroundline);
  // Tolerable angle difference to straight direction of Keeper to ball
  const float tolAngleToBall = fromDegrees(15.f);
  // Sidewards distance reach of a sidewards jump by Keeper
  const float reachOfJump = 1000.f;
  // length covered by sitDownKeeper on the floor by the robots
  const float sitdownBreadth = 360.f;
  // reaction time
  const float reactionTime = 2.f;

  const float middleConeTolerance = fromDegrees(20.f);
  const float maxConeAngle = fromDegrees(90.f);

  const Vector2<> middleOfGoal = {theFieldDimensions.xPosOwnGroundline + offsetKeeperOffLine, 0.f};
  float distanceKB = theBallModel.estimate.position.abs();

  Vector2<> gloBall = libWorldModel.ballPosition();
  Vector2<> relBall = gloToRel(gloBall);
  float ballAngle = atan2(relBall.y, relBall.x);

  float coneAngle = theRobotPose.rotation;

  bool wantsBall = theGameInfo.setPlay == SET_PLAY_NONE &&
                   ballInGoalBox(-theFieldDimensions.xPosOpponentPenaltyMark + theFieldDimensions.xPosOpponentGoalBox) &&
                   libCodeRelease.timeSinceBallWasSeen() < 500;

  // defines the sidewards distance a shot will pass by the keeper
  // positive values for left of keeper, negative for right of him
  // To add both angles we need to take the negative of the relative angle of the shot to the Keeper
  float sidewardsShotDist = 0.f;
  if (theBallModel.estimate.velocity.x < 0) {
    sidewardsShotDist = theBallModel.estimate.position.y - theBallModel.estimate.velocity.y /
                                                             theBallModel.estimate.velocity.x *
                                                             theBallModel.estimate.position.x;
  }

  float finalPosShoot = theRobotPose.translation.y + sidewardsShotDist;
  bool shootInside = (std::abs(finalPosShoot) <= theFieldDimensions.yPosLeftGoal) ? true : false;

  common_transition // play state just began
  {
    if (theRobotPose.timeSinceLastValid > 5000.f) {
      goto lost;
    }
    if ((theGameInfo.setPlay == SET_PLAY_GOAL_KICK ||
         (theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK && ballInGoalBox())) &&
        theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam &&
        !(theTeamMateData.isFullyActive[2] || theTeamMateData.isFullyActive[3])) {
      goto freeKick;
    } else if (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK) {
      goto penaltyKick;
    }
  }

  initial_state(walkToGoal) {
    transition {
      if (wantsBall && libCodeRelease.hasBallLock(wantsBall)) {
        goto kickBallAway;
      }
      if ((std::fabs(coneAngle) < maxConeAngle) && insideDefendingArea(offsetKeeperOffLine * 2)) {
        if (libCodeRelease.timeSinceBallWasSeen() < 1500.f && gloBall.x > theFieldDimensions.xPosOwnGroundline) {
          goto detectShot;
        }
        if (libCodeRelease.timeSinceBallWasSeen() > 1500.f) {
          goto searchForBall;
        }
      }
    }

    action {
      theHeadControlMode = HeadControl::walkScan;

      if ((std::fabs(coneAngle) > maxConeAngle) || !inGoalBox(theRobotPose.translation)) {
        // keeper somehow got out of penalty area or safe rotation, travel back to it
        TravelTo({0, middleOfGoal});
      } else if (insideDefendingArea(offsetKeeperOffLine * 2)) {
        // keeper reached defending area, turn towards the middle (we don't see the ball)
        WalkToTarget(Pose2D(0.4f, 0.f, 0.f), Pose2D(-theRobotPose.rotation, 0.f, 0.f));
      } else {
        // if keeper still in the penalty area we will walks backward to the center of the defending area.
        Vector2<> relTargetPos = gloToRel(middleOfGoal);
        WalkToTarget(Pose2D(0.0f, 1.0f, 1.0f), Pose2D(0.0f, relTargetPos));
      }
    }
  }

  state(searchForBall) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() < 500 && ballOnField()) {
        goto detectShot;
      }
      if ((std::fabs(coneAngle) > maxConeAngle) || !insideDefendingArea(offsetKeeperOffLine * 2)) {
        goto walkToGoal;
      }
    }
    action {
      if (theCombinedWorldModel.timeSinceBallLastSeen < 5000.f && theCombinedWorldModel.ballState.position.x < 0) {
        // Looks at team ball if its valid and in own half
        theHeadControlMode = HeadControl::lookAtBall;
        Stand();
      } else {
        if (std::fabs(coneAngle) > middleConeTolerance) {
          // Rotate towards the middle instead
          theHeadControlMode = HeadControl::walkScan;
          WalkToTarget(Pose2D(0.4f, 0.f, 0.f), Pose2D(-theRobotPose.rotation, 0.f, 0.f));
        } else {
          // At the middle already, scan left and right while standing
          theHeadControlMode = HeadControl::scanLeftRightKeeper;
          Stand();
        }
      }
    }
  }

  state(detectShot) {
    transition {
      // walk back to goal if not in penalty area
      if (!inGoalBox(theRobotPose.translation) || std::abs(coneAngle) > maxConeAngle + 20.f ||
          gloBall.x < theFieldDimensions.xPosOwnGroundline) {
        goto walkToGoal;
      }

      // distance between ball and keeper
      float distanceKB = theBallModel.estimate.position.abs();

      // upper limit to time to goal
      float timeToGoal = INFINITY;
      if (theBallModel.estimate.position.x > 0 && theBallModel.estimate.velocity.x < 0) {
        timeToGoal = -theBallModel.estimate.position.x / theBallModel.estimate.velocity.x;
      }

      // ball is CLOSE and moves FAST towards the goal, go try to save the shot
      if (distanceKB <= dangerousDistance && timeToGoal < reactionTime && std::abs(sidewardsShotDist) < reachOfJump &&
          shootInside && std::abs(sidewardsShotDist) > sitdownBreadth / 2.0 &&
          (lastSitDown != 0 && theFrameInfo.getTimeSince(lastSitDown) > 500.f)) {
        goto saveShot;
      }
      // we need to clear the ball
      if (wantsBall && libCodeRelease.hasBallLock(wantsBall)) {
        goto kickBallAway;
      }
      // look for ball if we do not see it
      if (libCodeRelease.timeSinceBallWasSeen() > 1500.f) {
        goto searchForBall;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;

      // walk on a semicircle if ball inside, otherwise go to it
      float radius = 650.f;
      if (distanceKB < radius) {
        radius = distanceKB;
      }

      float globalBallAngle = atan2(gloBall.y, (gloBall.x - theFieldDimensions.xPosOwnGroundline));
      float posX = Rangef(theFieldDimensions.xPosOwnGroundline + 150.f, theFieldDimensions.xPosOwnGoalBox * 1.05f)
                     .limit(theFieldDimensions.xPosOwnGroundline + radius * cos(globalBallAngle));
      float posY = radius * sin(globalBallAngle);
      const Vector2<> targetPos = {posX, posY};

      Vector2<> relTargetPos = gloToRel(targetPos);

      if (theFrameInfo.getTimeSince(lastSitDown) > 500.f &&
          ((theRobotPose.translation - targetPos).abs() > 50.f || std::fabs(ballAngle) > tolAngleToBall) &&
          gloBall.x > theFieldDimensions.xPosOwnGroundline) {
        // walk to the center of the goal looking in the direction of the ball with max angle
        lastSitDown = 0;
        if (relTargetPos.y > 20.f && (theRobotPose.translation.x - theFieldDimensions.xPosOwnGroundline) > 50.f) {
          WalkToTarget(Pose2D(0.2f, 0.f, 0.6f), Pose2D(ballAngle, 0.f, relTargetPos.y));
        } else {
          WalkToTarget(Pose2D(0.4f, 0.4f, 0.2f), Pose2D(ballAngle, relTargetPos.x, relTargetPos.y));
        }
      } else if (lastSitDown == 0) {
        // sit down if we are at good spot
        lastSitDown = theFrameInfo.time;
        SpecialAction(SpecialActionRequest::stand);
      }
    }
  }

  state(saveShot) {
    transition {
      if (action_done) {
        if (wantsBall && libCodeRelease.hasBallLock(wantsBall)) {
          dived = false;
          goto kickBallAway;
        } else if (dived) {
          dived = false;
          goto lookAround;
        } else {
          goto walkToGoal;
        }
      }
    }
    action {
      // angle the shot takes in regards to keeper assuming the keeper is STRAIGHTLY turned towards ball
      if (std::abs(sidewardsShotDist) < sitdownBreadth / 2.f) {
        SpecialAction(SpecialActionRequest::sitDownKeeper);
      } else if (sidewardsShotDist >= 0) {
        dived = true;
        SpecialAction(SpecialActionRequest::keeperJumpLeft, false); // jump left
      } else {
        dived = true;
        SpecialAction(SpecialActionRequest::keeperJumpLeft, true); // jump right
      }
    }
  }

  state(lookAround) {
    transition {
      if (state_time > 3000.f) {
        goto lost;
      }
      if (theRobotPose.timeSinceLastValid < 2000.f && state_time > 2000.f) {
        goto walkToGoal;
      }
    }
    action {
      theHeadControlMode = HeadControl::scanLeftRight;
      Stand();
    }
  }

  state(kickBallAway) {
    transition {
      if (!wantsBall || !libCodeRelease.hasBallLock(wantsBall)) {
        goto walkToGoal;
      }
    }
    action {
      const Vector2<> opponentGoal = Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);
      KickTo(opponentGoal, Pose2D(fromDegrees(60.f), 0.f, 0.f));
    }
  }

  state(lost) {
    transition {
      if (action_done) {
        goto walkToGoal;
      }
    }
    action { Relocate(); }
  }

  state(freeKick) {
    transition {
      if (theGameInfo.setPlay == SET_PLAY_NONE) {
        goto walkToGoal;
      }
    }
    action { KeeperFreeKick(); }
  }

  state(penaltyKick) {
    transition {
      if (theGameInfo.setPlay == SET_PLAY_NONE) {
        goto walkToGoal;
      }
    }
    action {
      if (theOwnTeamInfo.teamNumber != theGameInfo.kickingTeam) {
        PenaltyKeeper();
      } else {
        StandHigh();
      }
    }
  }
};

/********************* start defining functions ***************************/

// compute the euclidean distance between the points (x1,y1) and (x2,y2)

float dist(float x1, float y1, float x2, float y2) {
  return sqrtf(powf(x1 - x2, 2) + powf(y1 - y2, 2));
};

// This functions checks if the keeper is inside the defending area within the goal box.
bool insideDefendingArea(float xOffset) {
  float x = theRobotPose.translation.x;
  float y = theRobotPose.translation.y;

  if (x >= theFieldDimensions.xPosOwnGroundline && x <= theFieldDimensions.xPosOwnGroundline + xOffset &&
      std::abs(y) <= theFieldDimensions.yPosLeftGoal) {
    return true;
  } else {
    return false;
  }
};

// tells if a point is in the goal box
bool inGoalBox(Vector2<> point) {
  float x = point.x;
  float y = point.y;
  return x >= theFieldDimensions.xPosOwnGroundline && x <= theFieldDimensions.xPosOwnGoalBox &&
         std::abs(y) <= theFieldDimensions.yPosLeftGoalBox;
};

bool ballOnField() {
  const float safetyMargin = 200.f; // consider balls 20cm outside of field lines also valid
  return fabsf(theBallModel.estimate.position.x) <= fabsf(theFieldDimensions.xPosOpponentGroundline) + safetyMargin &&
         fabsf(theBallModel.estimate.position.y) <= fabsf(theFieldDimensions.yPosLeftSideline) + safetyMargin;
}

bool ballInGoalBox(float addTolX = 0.f) {
  // tolerance values away from penalty area in mm
  float tolX = 100.f + addTolX;
  float tolY = 100.f;
  // relative coordinates of the ball to keeper
  Vector2<> gloBall = libWorldModel.ballPosition();
  float x = gloBall[0];
  float y = gloBall[1];
  return x >= theFieldDimensions.xPosOwnGroundline && x <= theFieldDimensions.xPosOwnGoalBox + tolX &&
         std::abs(y) <= theFieldDimensions.yPosLeftGoalBox + tolY;
};

/************************* end defining functions *************************/
