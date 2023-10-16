/**
 * @file StandHigh.h
 *
 * Sets all members of the MotionRequest representation for simple standing
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(StandHigh) {
  /** Set the motion request. */
  initial_state(setRequest) {
    transition {
      if (action_done) {
        goto requestIsExecuted;
      }
    }
    action { SpecialAction(SpecialActionRequest::standHigh); }
  }

  /** The motion process has started executing the request. */
  target_state(requestIsExecuted) {
    transition {
      if (!action_done) {
        goto setRequest;
      }
    }
    action { SpecialAction(SpecialActionRequest::standHigh); }
  }
}
