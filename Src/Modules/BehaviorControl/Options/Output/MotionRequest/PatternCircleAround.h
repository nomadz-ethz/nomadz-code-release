/**
 * @file PatternCircleAround.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(PatternCircleAround, float globalTargetAngle) {
  Angle globalBallAngle =
    Angle(theRobotPose.rotation + libCodeRelease.computeAlignmentAngle(theBallModel.estimate.position)).normalize();
  Angle relAngle = Angle(globalTargetAngle - globalBallAngle).normalize();

  initial_state(setRequest) {
    transition {
      if (theMotionInfo.motionRequest.motion == MotionRequest::walk &&
          theMotionInfo.motionRequest.walkRequest.mode == WalkRequest::patternMode && state_time > 500) {
        goto requestIsExecuting;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      theArmMotionRequest.forceReturnDefault = false;

      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.inWalkKickRequest.kickDirection = static_cast<float>(relAngle);
      theMotionRequest.walkRequest.mode = WalkRequest::patternMode;
      theMotionRequest.walkRequest.inWalkKickRequest.kickType = InWalkKickRequest::circleAround;
    }
  }

  state(requestIsExecuting) {
    transition {
      if (action_done) {
        if (libCodeRelease.insidePatternCone(theBallModel.estimate.position, globalTargetAngle, 0.9)) {
          goto patternDone;
        } else {
          goto setRequest;
        }
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      theArmMotionRequest.forceReturnDefault = false;

      theMotionRequest.walkRequest.speed = Pose2D();
      theMotionRequest.walkRequest.mode = WalkRequest::patternMode;
      theMotionRequest.walkRequest.inWalkKickRequest.kickDirection = static_cast<float>(relAngle);
      theMotionRequest.walkRequest.inWalkKickRequest.kickType = InWalkKickRequest::circleAround;
    }
  }

  target_state(patternDone) {}
}
