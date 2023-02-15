/**
 * @file CircleAroundBall.h
 *
 *  (i.e. Circling around the ball until the global \c globalTargetAngle  angle is reached)
 * @param globalTargetAngle Global target angle
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(CircleAroundBall, Angle globalTargetAngle) {
  /** Set the motion request. */
  const Vector2<> gloBall = relToGlo(theBallModel.estimate.position);
  Angle relBallAngle = std::atan2(theBallModel.estimate.position.y, theBallModel.estimate.position.x);
  Angle globalBallAngle = theRobotPose.rotation + relBallAngle;
  Angle relAngle = globalTargetAngle - globalBallAngle;
  common_transition {
    if (libCodeRelease.timeSinceBallWasSeen() > 2000.f) {
      goto ballLost;
    }
    if (targetAngleRange.isInside(globalBallAngle - globalTargetAngle)) {
      goto targetAngleReached;
    }
  }

  initial_state(alignDistanceToTarget) {
    transition {
      if (targetDistanceRange.isInside(theBallModel.estimate.position.abs()) && targetAngleRange.isInside(relBallAngle)) {
        goto circleAroundTarget;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      float norminalTarget = (targetDistanceRange.min + targetDistanceRange.max) / 2;
      float distanceAdjustment = (theBallModel.estimate.position.abs() - norminalTarget) <= 0 ? -180 : 180;
      WalkAtSpeed(Pose2D(2 * relBallAngle, distanceAdjustment, 0.f));
    }
  }

  state(circleAroundTarget) {
    transition {
      if ((!targetDistanceRange.isInside(theBallModel.estimate.position.abs()) ||
           !targetAngleRange.isInside(relBallAngle)) &&
          state_time > 500) {
        goto alignDistanceToTarget;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      if (relAngle >= 0) {
        // counter-clockwise
        WalkAtSpeed(Pose2D(0.9, 0, -180));
      } else {
        // clockwise
        WalkAtSpeed(Pose2D(-0.9, 0, 180));
      }
    }
  }

  target_state(targetAngleReached) {}
  aborted_state(ballLost) {
    if (libCodeRelease.timeSinceBallWasSeen() < 300.f) {
      goto alignDistanceToTarget;
    }
  }
}
