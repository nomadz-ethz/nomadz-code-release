/**
 * @file StandWide.h
 *
 * Sets all members of the MotionRequest representation for simple standing
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(StandWide) {
  /** Set the motion request. */
  initial_state(setRequest) {
    transition {
      if (action_done) {
        goto requestIsExecuted;
      }
    }
    action { SpecialAction(SpecialActionRequest::standWide); }
  }

  /** The motion process has started executing the request. */
  target_state(requestIsExecuted) {
    transition {
      if (!action_done) {
        goto setRequest;
      }
    }
    action { SpecialAction(SpecialActionRequest::standWide); }
  }
}
