/**
 * @file LookAtReferee.h
 *
 * Sets all members of the MotionRequest representation for looking at the referee
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(LookAtReferee) {
  /** Set the motion request. */
  initial_state(setRequest) {
    transition {
      if (action_done) {
        goto requestIsExecuted;
      }
    }
    action { SpecialAction(SpecialActionRequest::stand); }
  }

  /** The motion process has started executing the request. */
  target_state(requestIsExecuted) {
    transition {
      if (!action_done) {
        goto setRequest;
      }
    }
    action { SpecialAction(SpecialActionRequest::stand); }
  }
}