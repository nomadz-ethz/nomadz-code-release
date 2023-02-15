/**
 * @file SupporterCoop.h
 *
 * The supported should be able to correctly position itself in order to be reached by a pass from behind,
 * and further support the striker actively.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

bool rotateBackDone = 0;
std::list<Range<float>> reducedRanges;

option(SupporterCoop) {

  const Vector2<> OpponentGoal(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);
  const Vector2<> OwnGoal(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosCenterGoal);
  Vector2<> gloBall = libWorldModel.ballPosition(2000.f);
  const Rangef rotationCone(-fromDegrees(5.f), fromDegrees(5.f)); // tolerance for positioning
  const Rangef insideField(theFieldDimensions.yPosRightPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);

  const Vector2<> goalMiddle = {theFieldDimensions.xPosOwnGroundline, 0};
  Rangef goalRange = {(goalMiddle - theRobotPose.translation).angle() - fromDegrees(45.f),
                      (goalMiddle - theRobotPose.translation).angle() + fromDegrees(45.f)};
  float goalDirection = (goalMiddle - theRobotPose.translation).angle();
  Rangef ballRange = {(gloBall - theRobotPose.translation).angle() - fromDegrees(45.f),
                      (gloBall - theRobotPose.translation).angle() + fromDegrees(45.f)}; // 90° range
  float ballDirection = (gloBall - theRobotPose.translation).angle();
  Rangef targetRange;
  float targetAngle;

  common_transition {
    if (libCodeRelease.timeSinceBallWasSeen() >
        5000.f) { // here by the robot itslef, otherwise can use "LastSeen" in WorldModel
      goto failure;
    }

    if (abs(gloBall.x) < theFieldDimensions.centerCircleRadius) {
      goto ballAtMidfield;
    }
  }

  initial_state(position) {
    transition {
      if (gloBall.x > 1.1 * theFieldDimensions.centerCircleRadius) {
        goto supportAttack;
      }
      if (!theTeamMateData.isActive[2] || !theTeamMateData.isActive[3]) { // they are penalized or crashed (?)
        goto supportDefense;
      }
      if (!rotateBackDone) {
        goto rotateBack;
      }
    }

    action {
      if (gloBall == Vector2<>(0, 0)) { // need to check what is this if noone see the ball
        lookTargetAngle = angleToGlobalTarget(OwnGoal);
        theHeadControlMode = HeadControl::lookAtTarget;
        targetRange = goalRange;
        targetAngle = goalDirection;
      } else {
        theHeadControlMode = HeadControl::lookAtBall;
        targetRange = ballRange;
        targetAngle = ballDirection;
      }

      float motionDirection = adjustPosition(targetRange, targetAngle);
      if ((theRobotPose.translation.y > theFieldDimensions.yPosLeftPenaltyArea && motionDirection < 0.f) ||
          (theRobotPose.translation.y < theFieldDimensions.yPosRightPenaltyArea && motionDirection > 0.f)) {
        motionDirection = 0;
      }

      WalkToTarget(Pose2D(0.f, 0.f, motionDirection), Pose2D(0.f, 0.f, 100.f * motionDirection));

      if (abs(theRobotPose.translation.x) > 500.f) {
        rotateBackDone = 0;
      }

      // the angle is computed in + and - direction starting from straight, so the 2 different posts are now problematic
    }
  }

  state(supportDefense) {
    transition {
      if (theTeamMateData.isActive[2] && theTeamMateData.isActive[3]) {
        rotateBackDone = 0;
        goto position;
      }
      if (gloBall.x > 1.1 * theFieldDimensions.centerCircleRadius) {
        goto supportAttack;
      }
    }

    action { DefenderPlay(25.f); }
  }

  state(supportAttack) {
    transition {
      if (gloBall.x < -1.1 * theFieldDimensions.centerCircleRadius) {
        rotateBackDone = 0;
        goto position;
      }
    }

    action {
      OffensivePlay(); // potentially here we could code a different logic, and use an additional state for OffensivePlay
                       // when ball is really close
    }
  }

  state(ballAtMidfield) {
    transition {
      if (gloBall.x > 1.1 * theFieldDimensions.centerCircleRadius) {
        goto supportAttack;
      }
      if (gloBall.x < -1.1 * theFieldDimensions.centerCircleRadius) {
        rotateBackDone = 0;
        goto position;
      }
    }

    action {
      OffensivePlay(); // here it's like a stricker, we want it to take the ball
    }
  }

  state(rotateBack) {
    transition {
      if (rotationCone.isInside(theRobotPose.rotation + pi) && theRobotPose.translation.x < 250.f) {
        rotateBackDone = 1;
        goto position;
      }
    }

    action {
      TravelTo(
        Pose2D(-pi,
               -50.f,
               insideField.limit(theRobotPose.translation.y))); // get towards own half, need to move it towards inner part
    }
  }

  aborted_state(failure) { ; }
}

// function to adjust position laterally in order to be well placed

float adjustPosition(Rangef targetRange, float targetAngle) {

  reducedRanges = libCodeRelease.calculateFreeRanges(theRobotPose.translation, targetRange, {1, 2, 3}); // result is a list

  Rangef bestRange = *(std::max_element(reducedRanges.begin(), reducedRanges.end(), sizeLarger));
  float motionTarget = bestRange.getCenter();

  if (motionTarget - targetAngle > fromDegrees(10.f)) {
    return 1.f;
  } else if (motionTarget - targetAngle < -fromDegrees(10.f)) {
    return -1.f;
  } else {
    return 0.f;
  }
}

static bool sizeLarger(Rangef a, Rangef b) {
  return a.getSize() < b.getSize();
}