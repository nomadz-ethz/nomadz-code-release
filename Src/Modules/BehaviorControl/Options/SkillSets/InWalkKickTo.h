/**
 * @file InWalkKickTo.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(InWalkKickTo, Angle gloBallTargetAngle) {

  Vector2<> gloBallPos = theRobotPose * theBallModel.estimate.position;
  Angle gloRobot2BallAngle = (gloBallPos - theRobotPose.translation).angle();
  // Angle gloBallTargetAngle = (gloBallTargetPos - gloBallPos).angle();
  Angle relBallTargetAngle = libCodeRelease.computeRelBallTargetAngle(gloBallTargetAngle);

  initial_state(approachBall) {
    transition {
      if (state_time > 500.f) {
        if (std::abs(relBallTargetAngle) < fromDegrees(91.f)) {
          if (libCodeRelease.insidePatternCone(theBallModel.estimate.position, gloRobot2BallAngle) && state_time > 500.f) {
            goto inWalkKickOmni;
          }
        } else {
          goto getBehind;
        }
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToTarget(Pose2D(1.f, 1.f, 0.f), Pose2D(theBallModel.estimate.position.angle(), theBallModel.estimate.position));
    }
  }

  state(getBehind) {
    transition {
      if (libCodeRelease.insidePatternCone(theBallModel.estimate.position, gloBallTargetAngle) && state_time > 500.f) {
        goto inWalkKickOmni;
      }
    }
    action { BallAlignment(gloBallTargetAngle); }
  }

  state(inWalkKickOmni) {
    transition {
      if (action_done) {
        goto approachBall;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      PatternInwalkKick(relBallTargetAngle, 0.5f);
    }
  }
}
