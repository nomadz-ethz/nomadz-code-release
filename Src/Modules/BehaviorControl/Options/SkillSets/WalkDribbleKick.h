/**
 * @file WalkDribbleKick.h
 *
 * Complete optimisable set of states that can be used by all player types to
 * walk towards the ball and kick it in the preferred direction. The ball is not
 * kicked if the robot is far from the opponent goal. It dribbles (walkIntoBall)
 * the ball until it is in range and then shoots. The side- and
 * backwards kick for example only have to be implemented once, here.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <algorithm>
#include <math.h>

option(WalkDribbleKick, Vector2<> kickTargetGlobal) {
  Vector2<> posTarget;        // Pop
  const float radius = 400.f; // r
  Vector2<> posGoalBall;      // Pgb
  Vector2<> posBall;          // B
  double targetAngle;
  double tangent;
  bool leftFootKick = true;
  float kickOffsetY = -75.f;
  const float yBound = 10.f;
  // static int rotDir = 1;
  const float fractionLimitDribble = 0.3; // robot will dribble only until fractionLimitDribble*length of opponent half.

  common_transition {
    // Recalculate best kicking foot (with some hysteresis)
    if (leftFootKick && theBallModel.estimate.position.y < -110.f) {
      leftFootKick = false;
      kickOffsetY = -kickOffsetY;
    } else if (!leftFootKick && theBallModel.estimate.position.y > 110.f) {
      leftFootKick = true;
      kickOffsetY = -kickOffsetY;
    }

    // FIXME: ball lock should not be calculated here

    if (theBallModel.estimate.position.abs() > 700.f) {
      goto walkToBall;
    }
  }

  initial_state(walkToBall) {
    transition {
      posBall = relToGlo(theBallModel.estimate.position);
      posGoalBall.x = posBall.x - kickTargetGlobal.x; // Xgb
      posGoalBall.y = posBall.y - kickTargetGlobal.y; // Ygb
      tangent = posGoalBall.y / posGoalBall.x;
      targetAngle = atan(tangent); // phi

      if ((angleToGlobalTarget(kickTargetGlobal) > 2.6 || angleToGlobalTarget(kickTargetGlobal) < -2.6) &&
          theBallModel.estimate.position.y <= 0 && theBallModel.estimate.position.x > 100.f) {
        posTarget.x = theBallModel.estimate.position.x;
        posTarget.y = theBallModel.estimate.position.y + 400.0;
        posTarget = relToGlo(posTarget);
      } else if ((angleToGlobalTarget(kickTargetGlobal) > 2.6 || angleToGlobalTarget(kickTargetGlobal) < -2.6) &&
                 theBallModel.estimate.position.y > 0 && theBallModel.estimate.position.x > 100.f) {
        posTarget.x = theBallModel.estimate.position.x;
        posTarget.y = theBallModel.estimate.position.y - 400.0;
        posTarget = relToGlo(posTarget);
      } else {
        posTarget.x = kickTargetGlobal.x + posGoalBall.x - radius * cos(targetAngle);
        posTarget.y = kickTargetGlobal.y + posGoalBall.y;
      }

      if (gloToRel(posTarget).abs() < 300.f) {
        goto alignToTarget;
      }
    }

    action {
      theHeadControlMode = HeadControl::lookAtBall;
      // Walk to a target behind the ball, not to be ball itself
      //            Vector2<> walkToBallTarget = theBallModel.estimate.position;
      //            //-ownBallToTargetRel(kickTargetGlobal).normalize(120);
      // walkToBallTarget = gloToRel(walkToBallTarget);
      // WalkRRT(walkToBallTarget);
      posBall = relToGlo(theBallModel.estimate.position);
      posGoalBall.x = posBall.x - kickTargetGlobal.x; // Xgb
      posGoalBall.y = posBall.y - kickTargetGlobal.y; // Ygb
      tangent = posGoalBall.y / posGoalBall.x;
      targetAngle = atan(tangent); // phi
      if ((angleToGlobalTarget(kickTargetGlobal) > 2.6 || angleToGlobalTarget(kickTargetGlobal) < -2.6) &&
          theBallModel.estimate.position.y < 0 && theBallModel.estimate.position.x > 100.f) {
        posTarget.x = theBallModel.estimate.position.x;
        posTarget.y = theBallModel.estimate.position.y + 400.0;
        posTarget = relToGlo(posTarget);
      } else if ((angleToGlobalTarget(kickTargetGlobal) > 2.6 || angleToGlobalTarget(kickTargetGlobal) < -2.6) &&
                 theBallModel.estimate.position.y > 0 && theBallModel.estimate.position.x > 100.f) {
        posTarget.x = theBallModel.estimate.position.x;
        posTarget.y = theBallModel.estimate.position.y - 400.0;
        posTarget = relToGlo(posTarget);
      } else {
        posTarget.x = kickTargetGlobal.x + posGoalBall.x - radius * cos(targetAngle);
        posTarget.y = kickTargetGlobal.y + posGoalBall.y;
      }
      WalkToTarget(Pose2D(0.9f, 0.9f, 0.f), Pose2D(angleToGlobalTarget(posTarget), gloToRel(posTarget)));
    }
  }

  state(alignToTarget) {
    transition {
      //      if(std::abs(angleToGlobalTarget(kickTargetGlobal)) < fromDegrees(12.f) && theRobotPose.translation.x <
      //      (theFieldDimensions.xPosOpponentGroundline-2500))
      //          goto walkIntoBall;

      // TODO Check if also behind ball / ball still visible! (Avoid cases where walked in front of ball, thus not seeing it
      // anymore)

      if (std::abs(angleToGlobalTarget(kickTargetGlobal)) < fromDegrees(20.f) &&
          (std::abs(theBallModel.estimate.position.y) < 240.f)) {
        if (theRobotPose.translation.x > theFieldDimensions.xPosOpponentGroundline * fractionLimitDribble) {
          goto alignBehindBall;
        } else {
          goto walkIntoBall;
        }
      }
    }

    action {
      theHeadControlMode = HeadControl::lookAtBall;
      posBall = relToGlo(theBallModel.estimate.position);

      const Vector2<> ballToGoal = kickTargetGlobal - posBall;
      targetAngle = ballToGoal.angle();
      posTarget = posBall - ballToGoal.direction() * radius;
      float relativeAngle = targetAngle - theRobotPose.rotation;
      if (relativeAngle > pi) {
        relativeAngle -= 2 * pi;
      }
      if (relativeAngle <= -pi) {
        relativeAngle += 2 * pi;
      }

      if (theBallModel.estimate.position.y < 0 && relativeAngle > 0) {
        WalkToTarget(Pose2D(0.6f, 0.78f, 0.8f),
                     Pose2D(std::max(-pi + 0.01f, -2.f * pi + relativeAngle), gloToRel(posTarget))); // hack
      } else if (theBallModel.estimate.position.y > 0 && relativeAngle < 0) {
        WalkToTarget(Pose2D(0.6f, 0.78f, 0.8f),
                     Pose2D(std::min(pi - 0.01f, 2.f * pi + relativeAngle), gloToRel(posTarget))); // hack
      } else {
        WalkToTarget(Pose2D(0.6f, 0.78f, 0.8f), Pose2D(relativeAngle, gloToRel(posTarget)));
      }
      //      WalkToTarget(Pose2D(1.f, 0.1f, 0.5f), Pose2D(angleToGlobalTarget(kickTargetGlobal),
      //                    theBallModel.estimate.position.x - 220.f,
      //                    theBallModel.estimate.position.y));
      //      WalkAtSpeed(Pose2D(rotDir*0.5f, 0.f, -rotDir*30.f));

      //      WalkAtSpeed(Pose2D(rotDir*0.25f+0.75f*theBallModel.estimate.position.angle(),
      //      0.65f*(theBallModel.estimate.position.x-220.f) , -rotDir*40.f));
    }
  }

  /*
  state(alignForDribble)
  {
      transition
      {

          if (std::abs(angleToGlobalTarget(kickTargetGlobal)) < fromDegrees(60.f)) //&& theRobotPose.translation.x <
  (theFieldDimensions.xPosOpponentGroundline-2500))
              goto walkIntoBall;
          //      else if(std::abs(angleToGlobalTarget(kickTargetGlobal)) < fromDegrees(60.f)
          //             && (std::abs(theBallModel.estimate.position.y) < 240.f))
          //           goto alignBehindBall;
      }

      action
      {
          theHeadControlMode = HeadControl::lookAtBall;
          //      WalkToTarget(Pose2D(1.f, 0.1f, 0.5f), Pose2D(angleToGlobalTarget(kickTargetGlobal),
          //                    theBallModel.estimate.position.x - 220.f,
          //                    theBallModel.estimate.position.y));
          //      WalkAtSpeed(Pose2D(rotDir*0.5f, 0.f, -rotDir*30.f));

          WalkAtSpeed(Pose2D(rotDir * 0.25f + 0.75f * theBallModel.estimate.position.angle(), 0.65f *
  (theBallModel.estimate.position.x - 220.f), -rotDir * 40.f));
      }
  }
  */

  state(alignBehindBall) {
    transition {

      if (libCodeRelease.between(theBallModel.estimate.position.y + kickOffsetY, -yBound, yBound) &&
          libCodeRelease.between(theBallModel.estimate.position.x, 170.f, 200.f) &&
          std::abs(angleToGlobalTarget(kickTargetGlobal)) < fromDegrees(7.5f)) {
        goto kick;
      }
    }

    action {
      // if (theBallModel.estimate.position.abs() < 300.f)
      // {
      theHeadControlMode = HeadControl::lookDown;
      WalkToTarget(Pose2D(0.1f, 0.78f, 1.f),
                   Pose2D(angleToGlobalTarget(kickTargetGlobal),
                          theBallModel.estimate.position.x - 170.f,
                          theBallModel.estimate.position.y + kickOffsetY));
      // }
    }
  }

  /* check whether ball is still seen & then kick */
  state(prepareKick) {

    transition {
      // if (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 500)
      goto kick;
      // else
      //     SearchForBall();
    }
  }

  state(kick) {
    transition {
      if (state_time > 3000) {
        goto success;
      }
    }

    action { PowerKick(leftFootKick); }
  }

  /** kick is done. */
  target_state(success) { ; }

  /* kick aborted: because ball not seen anymore */
  aborted_state(failure) { ; }

  state(walkIntoBall) {
    transition {
      if (state_time > 50000) {
        goto failure;
      } else if (std::abs(theBallModel.estimate.position.y) > 240.f || theBallModel.estimate.position.x < 100.f) {
        goto alignToTarget;
      } else if (theRobotPose.translation.x > theFieldDimensions.xPosOpponentGroundline *
                                                fractionLimitDribble) { // can be changed if the field has less friction
        goto alignBehindBall;
      }
    }

    action {
      WalkToTarget(Pose2D(0.5f, 0.5f, 1.f),
                   Pose2D(angleToGlobalTarget(kickTargetGlobal),
                          theBallModel.estimate.position.x + 20.f,
                          theBallModel.estimate.position.y));
    }
  }
}
