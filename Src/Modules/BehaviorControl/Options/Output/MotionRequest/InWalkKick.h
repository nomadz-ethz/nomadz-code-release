/**
 * @file InWalkKick.h
 *
 * Kicks the Ball while executing the current walk with the specified \c stepRequest.
 * @param stepRequest The WalkRequest::StepRequest to be executed
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(InWalkKick, WalkRequest::StepRequest stepRequest) {
  /** Set the motion request / stepRequest. */
  initial_state(launch) {
    transition {
      if (theMotionInfo.motionRequest.motion == MotionRequest::walk &&
          theMotionInfo.motionRequest.walkRequest.stepRequest == stepRequest) {
        goto execute;
      }
    }
    action {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.stepRequest = stepRequest;
    }
  }

  /** Executes the kick */
  state(execute) {
    transition {
      if (theMotionInfo.motionRequest.walkRequest.stepRequest == WalkRequest::none) {
        goto finished;
      }
    }
    action {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.stepRequest = WalkRequest::none;
    }
  }

  /** The kick has been executed */
  target_state(finished) {
    action {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.stepRequest = WalkRequest::none;
    }
  }
}
