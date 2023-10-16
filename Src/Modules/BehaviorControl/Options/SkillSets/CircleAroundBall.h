/**
 * @file CircleAroundBall.h
 *
 *  (i.e. Circling around the ball until the global \c globalTargetAngle  angle is reached)
 * @param globalTargetAngle Global target angle
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(CircleAroundBall, Angle globalTargetAngle, Angle angleAlignmentTolerance = fromDegrees(20.f)) {
  /** Set the motion request. */
  const Vector2<> gloBall = relToGlo(theBallModel.estimate.position);
  Angle relBallAngle = theBallModel.estimate.position.angle();
  Angle globalBallAngle = theRobotPose.rotation + relBallAngle;
  Angle relAngle = globalTargetAngle - globalBallAngle;
  common_transition {
    if (std::abs(globalBallAngle - globalTargetAngle) < angleAlignmentTolerance) {
      goto targetAngleReached;
    }
  }

  initial_state(alignDistanceToTarget) {
    transition {
      if (targetDistanceRange.isInside(theBallModel.estimate.position.abs()) &&
          std::abs(relBallAngle) < angleAlignmentTolerance) {
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
           (std::abs(relBallAngle) > angleAlignmentTolerance)) &&
          state_time > 500) {
        goto alignDistanceToTarget;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      if (relAngle >= 0) {
        // counter-clockwise
        WalkAtSpeed(Pose2D(1.0, 0, -180));
      } else {
        // clockwise
        WalkAtSpeed(Pose2D(-1.0, 0, 180));
      }
    }
  }

  target_state(targetAngleReached) {}
}
