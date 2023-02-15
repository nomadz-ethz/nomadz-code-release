/**
 * @file WalkAndKick.h
 *
 * Complete optimizable set of states that can be used by all player types to
 * walk towards the ball and kick it in the prefered direction. The side- and
 * backwards kick for example only have to be implemented once, here.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <algorithm>

option(WalkAndKick, Vector2<> kickTargetGlobal) {
  const float tolAngleToBall = fromDegrees(15.f);

  Vector2<> posTarget;   // Pop
  double radius = 400.0; // r
  Vector2<> posGoalBall; // Pgb
  Vector2<> posBall;     // B
  double targetAngle;
  double tangent;
  bool leftFootKick = true;
  float kickOffsetY = -75.f;
  const float yBound = 10.f;
  // static int rotDir = 1;

  common_transition {
    // FIXME: ball lock should not be implemented here
    if (theBallModel.estimate.position.abs() > 600.f) {
      // WalkRRT(theBallModel.estimate.position);
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
      if (gloToRel(posTarget).abs() < 100.f) {
        if (theBallModel.estimate.position.y < 0) {
          leftFootKick = false;
          kickOffsetY = 75.f;
        } else {
          leftFootKick = true;
          kickOffsetY = -75.f;
        }
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

      if (std::abs(angleToGlobalTarget(kickTargetGlobal)) < fromDegrees(20.f) &&
          (std::abs(theBallModel.estimate.position.y) < 240.f)) {
        goto alignBehindBall;
      }
    }

    action {
      theHeadControlMode = HeadControl::lookAtBall;
      posBall = relToGlo(theBallModel.estimate.position);
      tangent = posGoalBall.y / posGoalBall.x;

      posGoalBall = kickTargetGlobal - posBall;
      targetAngle = posGoalBall.angle();
      posTarget = posBall - posGoalBall / posGoalBall.abs() * radius;
      float relativeAngle = targetAngle - theRobotPose.rotation;
      if (relativeAngle > pi) {
        relativeAngle -= 2 * pi;
      }
      if (relativeAngle <= -pi) {
        relativeAngle += 2 * pi;
      }

      if (theBallModel.estimate.position.y < 0 && relativeAngle > 0) {
        WalkToTarget(Pose2D(0.6f, 0.78f, 0.8f),
                     Pose2D(fmax(-pi + 0.01, -2 * pi + relativeAngle), gloToRel(posTarget))); // hack
      } else if (theBallModel.estimate.position.y > 0 && relativeAngle < 0) {
        WalkToTarget(Pose2D(0.6f, 0.78f, 0.8f),
                     Pose2D(fmin(pi - 0.01, 2 * pi + relativeAngle), gloToRel(posTarget))); // hack
      } else {
        WalkToTarget(Pose2D(0.6f, 0.78f, 0.8f), Pose2D(relativeAngle, gloToRel(posTarget)));
      }
      //      WalkToTarget(Pose2D(1.f, 0.1f, 0.5f), Pose2D(angleToGlobalTarget(kickTargetGlobal),
      //										theBallModel.estimate.position.x - 220.f,
      //										theBallModel.estimate.position.y));
      //      WalkAtSpeed(Pose2D(rotDir*0.5f, 0.f, -rotDir*30.f));

      //      WalkAtSpeed(Pose2D(rotDir*0.25f+0.75f*theBallModel.estimate.position.angle(),
      //      0.65f*(theBallModel.estimate.position.x-220.f) , -rotDir*40.f));
    }
  }

  state(alignBehindBall) {
    transition {
      if (libCodeRelease.between(theBallModel.estimate.position.y + kickOffsetY, -yBound, yBound) &&
          libCodeRelease.between(theBallModel.estimate.position.x, 170.f, 190.f)) {
        //&& std::abs(angleToGlobalTarget(kickTargetGlobal)) < fromDegrees(15.f))
        goto prepareKick;
      }
    }
    action {
      //      if (theBallModel.estimate.position.abs() < 300.f)
      //      {
      theHeadControlMode = HeadControl::lookDown;
      WalkToTarget(Pose2D(0.f, 0.78f, 1.f),
                   Pose2D(angleToGlobalTarget(kickTargetGlobal),
                          theBallModel.estimate.position.x - 180.f,
                          theBallModel.estimate.position.y + kickOffsetY));
      //      }
    }
  }

  /* check whether ball is still seen & then kick */
  state(prepareKick) {
    transition {
      //      if(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 500)
      goto kick;
      //      else
      //        SearchForBall();
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

  state(turnToBall) {
    transition {

      if (std::abs(theBallModel.estimate.position.angle()) < tolAngleToBall) {
        // if (angleToGlobalTarget(kickTargetGlobal) < 0)
        //   rotDir = -1;
        // else
        //   rotDir = 1;
        goto alignToTarget;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToTarget(Pose2D(0.5f, 0.5f, 0.5f), Pose2D(theBallModel.estimate.position.angle(), 0.f, 0.f));
    }
  }
}
