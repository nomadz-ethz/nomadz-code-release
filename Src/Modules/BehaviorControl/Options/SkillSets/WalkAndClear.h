/**
 * @file WalkAndClear.h
 *
 * Complete optimizable set of states that can be used by defenders to
 * walk towards the ball and clear it.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#define yBound 10.f

bool cleared = false;
option(WalkAndClear) {
  bool leftFootKick;
  Vector2<> ballPos = relToGlo(theBallModel.estimate.position);
  Vector2<> target = ballPos;

  common_transition {
    if (theBallModel.estimate.position.abs() > 400.f)
      goto walkToBall;
  }

  initial_state(walkToBall) {
    transition {
      if ((theBallModel.estimate.position.abs() < 300.f))
        goto clear;
    }
    action {

      theHeadControlMode = HeadControl::lookBallGoal;
      TravelTo(theBallModel.estimate.position);
      // WalkRRT(theBallModel.estimate.position);
    }
  }

  state(clear) {
    transition {
      if (libCodeRelease.between(theRobotPose.translation.x, opponentGoalLocation.x, ballPos.x))
        target.y = sgn(ballPos.y) * theFieldDimensions.yPosLeftFieldBorder;
      else
        target = {theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal};
      if (angleToGlobalTarget(target) > 0) {
        leftFootKick = false;
        kickOffsetY = 45.f;
        kickOffsetYmin = -kickOffsetY - yBound;
        kickOffsetYmax = -kickOffsetY + yBound;
      } else {
        leftFootKick = true;
        kickOffsetY = -45.f;
        kickOffsetYmin = -kickOffsetY - yBound;
        kickOffsetYmax = -kickOffsetY + yBound;
      }
      goto alignBehindBall;
    }
  }

  state(alignBehindBall) {
    transition {
      if (libCodeRelease.between(theBallModel.estimate.position.y, kickOffsetYmin, kickOffsetYmax) &&
          (theBallModel.estimate.position.x < 150.f) && std::abs(angleToGlobalTarget(target)) < fromDegrees(10.f))
        PowerKick(leftFootKick);
    }
    action {
      theHeadControlMode = HeadControl::lookDown;
      WalkToTarget(Pose2D(3.f, 3.f, 3.f),
                   Pose2D(angleToGlobalTarget(target),
                          theBallModel.estimate.position.x - 70.f,
                          theBallModel.estimate.position.y + kickOffsetY));
    }
  }
}
