/**
 * @file BallAlignment.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(BallAlignment, Angle globBallDirection) {
  Angle relBallDirection = Angle(globBallDirection - theRobotPose.rotation).normalize();

  const Vector2<> gloBall = relToGlo(theBallModel.estimate.position);
  const float ballAngle = (gloBall - theRobotPose.translation).angle();
  const Pose2D targetFrame(ballAngle,
                           gloBall); // global
  const float ballAlignmentDistance = 800.f;
  const float transitionMargin = 50.f;

  common_transition {
    if (libCodeRelease.insidePatternCone(theBallModel.estimate.position, globBallDirection, 0.9)) {
      goto success;
    }
  }

  initial_state(rotateToBall) {
    transition {
      if (theBallModel.estimate.position.abs() > ballAlignmentDistance + transitionMargin) {
        goto travelToBall;
      }
      if (libCodeRelease.ballInsideAlignmentCircle(theBallModel.estimate.position)) {
        goto closeRangeAlignment;
      }
    }
    action {
      WalkToTarget(Pose2D(1.f, 0.6f, 0.f), Pose2D(theBallModel.estimate.position.angle(), theBallModel.estimate.position));
    }
  }

  state(travelToBall) {
    transition {
      if (theBallModel.estimate.position.abs() < ballAlignmentDistance) {
        goto rotateToBall;
      }
    }
    action { TravelTo(targetFrame); }
  }

  state(closeRangeAlignment) {
    transition {
      if (!libCodeRelease.ballInsideAlignmentCircle(theBallModel.estimate.position, 550.f, 650.f)) {
        goto rotateToBall;
      }
      if (theBallModel.estimate.position.abs() < targetDistanceRange.max) {
        goto circleAround;
      }
    }
    action {
      if (theBallModel.estimate.position.y * relBallDirection > 0) {
        WalkToTarget(Pose2D(1.f, 1.f, 0.f), Pose2D(theBallModel.estimate.position.angle(), theBallModel.estimate.position));
      } else {
        WalkToTarget(Pose2D(0.f, 1.f, 1.f), Pose2D(0, theBallModel.estimate.position + Vector2<>(-150.f, 0.f)));
      }
    }
  }

  state(circleAround) {
    transition {}
    action { PatternCircleAround(globBallDirection); }
  }

  target_state(success) {}
};