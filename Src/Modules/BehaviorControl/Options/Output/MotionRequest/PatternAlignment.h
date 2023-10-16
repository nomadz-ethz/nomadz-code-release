/**
 * @file PatternAlignment.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(PatternAlignment, float relAngle) {
  /** Set the motion request. */
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
      theMotionRequest.walkRequest.inWalkKickRequest.kickDirection = relAngle;
      theMotionRequest.walkRequest.mode = WalkRequest::patternMode;
      theMotionRequest.walkRequest.inWalkKickRequest.kickType = InWalkKickRequest::alignment;
    }
  }

  state(requestIsExecuting) {
    transition {
      if (thePlannedSteps.isLeavingPossible == true && state_time > 100) {
        if ((std::abs(theBallModel.estimate.position.y - 55.f) < 50 ||
             std::abs(theBallModel.estimate.position.y + 55.f) < 50)) {
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
      theMotionRequest.walkRequest.inWalkKickRequest.kickDirection = relAngle;
      theMotionRequest.walkRequest.inWalkKickRequest.kickType = InWalkKickRequest::alignment;
    }
  }

  target_state(patternDone) {}
}
