/**
 * @file OffensivePlay.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(OffensivePlay) {
  const Vector2<> gloOpponentGoal(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);
  Vector2<> gloBall = libWorldModel.ballPosition(2000.f);
  bool ballLocationInRange =
    (theRobotInfo.number % 2 == 0)
      ? gloBall.x > theFieldDimensions.xPosOwnGroundline * 0.2 && gloBall.x > theFieldDimensions.xPosOwnPenaltyArea
      : gloBall.x > theFieldDimensions.xPosOwnGroundline * 0.1 && gloBall.x > theFieldDimensions.xPosOwnPenaltyArea;

  const int otherOffensivePlayer = (theRobotInfo.number == 4) ? 5 : 4;
  bool wantsBall = ballLocationInRange && libCodeRelease.timeSinceBallWasSeen() < 2500;
  const float activeExtraBallScore = -300.f;
  float sideBallScore = 0.f;
  if (gloBall.x < theRobotPose.translation.x - 200.f) {
    sideBallScore = 1000.f;
  }

  common_transition {
    // NOTE: ball score will be set lower if in wait for ball state
    if (!libCodeRelease.hasBallLock(wantsBall, activeExtraBallScore + sideBallScore)) {
      goto waitForBall;
    }
  }

  initial_state(play) {
    transition {
      if (theBallModel.timeSinceLastSeen > ballLostTime || theRobotPose.timeSinceLastValid > robotPoseLostTime) {
        goto failure;
      }
      if ((gloBall - gloOpponentGoal).abs() > 3750.f) {
        goto dribble;
      }
    }
    action {
      float nearestPlayerInSight = libCodeRelease.nearestPlayerInSight(0.8, 500);

      if (shouldRush()) {
        // ScoreGoal takes care of rushing
        ScoreGoal();
      } else if (nearestPlayerInSight < 500.f) {
        DribbleTo(gloOpponentGoal, fromDegrees(30.f));
      } else if (nearestPlayerInSight < 750.f) {
        DribbleTo(gloOpponentGoal, fromDegrees(15.f));
      } else {
        ScoreGoal();
      }
    }
  }

  state(dribble) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() > 5000.f) {
        goto failure;
      }
      if ((gloBall - gloOpponentGoal).abs() < 3250.f) {
        goto play;
      }
    }
    action {
      if (relToGlo(theBallModel.estimate.position).y > -500) {
        DribbleTo(Vector2<>(2200.f, 2000.f), fromDegrees(30.f));
      } else {
        DribbleTo(Vector2<>(2200.f, -2000.f), fromDegrees(30.f));
      }
    }
  }

  state(waitForBall) {
    transition {
      // If the ball was not seen after a certain time and no search is in progress search for ball
      if ((libCodeRelease.timeSinceBallWasSeen() > 5000.f &&
           (theCombinedWorldModel.timeSinceBallLastSeenPlayer[otherOffensivePlayer] > 5000.f ||
            (theTeamMateData.robotPoses[otherOffensivePlayer].translation - gloBall).abs() > 1500.f)) ||
          libCodeRelease.timeSinceBallWasSeen() > 15000) {
        goto failure;
      }
      if (libCodeRelease.hasBallLock(wantsBall, sideBallScore)) {
        goto play;
      }
    }
    action {
      float interceptX;
      float interceptY;
      Vector2<> otherOffensivePlayerPosition = theTeamMateData.robotPoses[otherOffensivePlayer].translation;
      if (gloBall.x > 0) {
        // If the ball is on the opponent side, go behind the player with the ball and follow him
        interceptX = otherOffensivePlayerPosition.x - 1000.f;

        interceptY = (theFieldDimensions.xPosOpponentGroundline - interceptX) /
                     (theFieldDimensions.xPosOpponentGroundline - otherOffensivePlayerPosition.x) *
                     otherOffensivePlayerPosition.y;

        if (interceptY > theRobotPose.translation.y) {
          interceptY = interceptY - 750.f;
        } else {
          interceptY = interceptY + 750.f;
        }
      } else {
        // If the ball is on our side, go more forward and wait for the ball to come to your side by standing between the
        // goal and the ball
        interceptX = (theRobotInfo.number % 2 == 0) ? theFieldDimensions.xPosOpponentGroundline * 0.2f
                                                    : theFieldDimensions.xPosOpponentGroundline * 0.4f;
        interceptY = (theFieldDimensions.xPosOpponentGroundline - interceptX) /
                     (theFieldDimensions.xPosOpponentGroundline - gloBall.x) * gloBall.y;
        // Add an offset such that the robot would not stand in the way
        if (interceptY > 0) {
          interceptY = interceptY - 1500.0;
        } else {
          interceptY = interceptY + 1500.0;
        }
      }
      Vector2<> interceptPoint;
      if (interceptY > theFieldDimensions.yPosLeftFieldBorder - 1000.f) {
        interceptPoint = {interceptX, theFieldDimensions.yPosLeftFieldBorder - 1000.f};
      } else if (interceptY < theFieldDimensions.yPosRightFieldBorder + 1000.f) {
        interceptPoint = {interceptX, theFieldDimensions.yPosRightFieldBorder + 1000.f};
      } else {
        interceptPoint = {interceptX, interceptY};
      }

      theHeadControlMode = HeadControl::lookAtBall;
      float angle = (gloBall - theRobotPose.translation).angle();
      TravelTo(Pose2D(angle, interceptPoint));
    }
  }

  aborted_state(failure) { ; }
}
