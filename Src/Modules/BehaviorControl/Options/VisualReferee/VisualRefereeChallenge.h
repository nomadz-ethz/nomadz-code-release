/**
 * @file VisualRefereeChallenge.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iostream>
#include <vector>

int conf_threshold = 85; // threshold to react directly in %

int signal_current = -1;
int signal_max_conf = -1;
int confidence_current = 0;
int confidence_max = 0;

option(VisualRefereeChallenge) {
  common_transition {
    if (theKeyStates.pressed[KeyStates::headFront]) {
      goto prepare;
    }
  }

  initial_state(initial) {
    transition {
      if (action_done) {
        goto prepare;
      }
    }
    action { ButtonPressedAndReleased(KeyStates::chest, 1000, 0); }
  }

  // prepare: get ready for the signal
  state(prepare) {
    transition {
      if (action_done) {
        goto ready;
      }
    }
    action {
      // initialization
      signal_current = -1;
      signal_max_conf = -1;
      confidence_current = 0;
      confidence_max = 0;

      // initial position of robot (important that head angle correct so referee in frame)
      SpecialAction(SpecialActionRequest::Ref_StandInit);
    }
  }

  // ready: wait for the whistle
  state(ready) {
    transition {
      bool whistle_detected = theWhistle.whistleDetected;
      // bool whistle_detected = true; // for testing

      if (whistle_detected) {
        goto heard;
      } else if (state_time > 8000) {
        // if whistle isn't detected after 8sec change state anyways
        goto set;
      }
    }
  }

  // heard: wait for one second
  state(heard) {
    transition {
      if (state_time > 1000) {
        // wait one second until referee starts showing signal
        goto set;
      }
    }
  }

  // set: wait until the signal is detected
  state(set) {
    transition {
      if (state_time > 20000) {
        // time is over to make prediction
        goto prepare;
      }

      else if (state_time > 11000 && signal_max_conf > 0) {
        // after 10 seconds referee stops showing signal
        // make prediction of most likely signal over past 11 seconds (1 second added)
        signal_current = signal_max_conf; // signal_current used in react state
        goto react;
      }

      else if (signal_current > 0 && confidence_current > conf_threshold) {
        // make prediction directly if confidence is over 90%
        goto react;
      }
    }
    action {
      // get current signal and confidence from representation
      signal_current = theRefereePercept.signal;
      confidence_current = theRefereePercept.confidence;

      // update most likely signal
      if (confidence_current > confidence_max) {
        signal_max_conf = signal_current;
        confidence_max = confidence_current;
      }
    }
  }

  // react: mirror the signal and announce it (until front head button pressed)
  state(react) {
    action { VisualRefereeReact(signal_current); }
  }
}
