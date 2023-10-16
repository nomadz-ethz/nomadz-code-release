/**
 * @file PatternInwalkKick.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(PatternInwalkKick, float relAngle, float kickStrength = 0.6f) {
  /** Set the motion request. */
  initial_state(setRequest) {
    transition {
      if (theMotionInfo.motionRequest.motion == MotionRequest::walk &&
          theMotionInfo.motionRequest.walkRequest.mode == WalkRequest::patternMode && state_time > 500) {
        goto patternDone;
      }
    }
    action {

      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.inWalkKickRequest.kickDirection = relAngle;
      theMotionRequest.walkRequest.inWalkKickRequest.kickStrength = kickStrength;
      theMotionRequest.walkRequest.mode = WalkRequest::patternMode;
      theMotionRequest.walkRequest.inWalkKickRequest.kickType = InWalkKickRequest::omniKick;
    }
  }
  state(patternDone) {
    transition {
      if (thePlannedSteps.isLeavingPossible == true && state_time > 250) {
        const Vector2<> inWalkKickOffset = Vector2<>(1500.f, 0.f);
        gloKickEstimate =
          Pose2D(Angle(theRobotPose.rotation + relAngle).normalize(), theRobotPose.translation) * inWalkKickOffset;
        ballKickedAway = true;

        goto success;
      }
    }
    action {
      WalkToTarget(Pose2D(1.f, 1.f, 0.f), Pose2D(theBallModel.estimate.position.angle(), theBallModel.estimate.position));
    }
  }
  /** The motion process has started executing the request. */
  target_state(success) {}
}
