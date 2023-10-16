/**
 * @file SpecialAction.h
 *
 * Sets all members of the MotionRequest representation for executing a SpecialAction
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(SpecialAction, SpecialActionRequest::SpecialActionID id, bool mirror = false) {
  /** Set the motion request. */
  initial_state(setRequest) {
    transition {
      if (theMotionInfo.motionRequest.motion == MotionRequest::specialAction &&
          theMotionInfo.motionRequest.specialActionRequest.specialAction == id &&
          theMotionInfo.motionRequest.specialActionRequest.mirror == mirror) {
        goto requestIsExecuted;
      }
    }
    action {
      theMotionRequest.motion = MotionRequest::specialAction;
      theMotionRequest.specialActionRequest.specialAction = id;
      theMotionRequest.specialActionRequest.mirror = mirror;
    }
  }

  /** The motion process has started executing the request. */
  target_state(requestIsExecuted) {
    transition {
      if (theMotionInfo.motionRequest.motion != MotionRequest::specialAction ||
          theMotionInfo.motionRequest.specialActionRequest.specialAction != id ||
          theMotionInfo.motionRequest.specialActionRequest.mirror != mirror) {
        goto setRequest;
      }
    }
    action {
      theMotionRequest.motion = MotionRequest::specialAction;
      theMotionRequest.specialActionRequest.specialAction = id;
      theMotionRequest.specialActionRequest.mirror = mirror;
    }
  }
}
