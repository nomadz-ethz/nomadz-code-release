/**
 * @file VisualRefereeChallenge.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include <iostream>
#include <vector>

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

  state(prepare) {
    transition {
      if (action_done) {
        goto ready;
      }
    }
    action {
      // initial position of robot (important that head angle correct so referee in frame)
      SpecialAction(SpecialActionRequest::Ref_StandInit);
    }
  }

  state(ready) {
    transition {
      if (state_time > 3000) {
        theRefereePercept.runModel = true;
      }
      if (state_time > 8000) {
        goto react;
      }
    }
  }

  state(react) {
    transition {
      if (theRefereePercept.signal > 0) {
        OUTPUT_TEXT("PREDICTED: " << theRefereePercept.signal);
        bool play_sound = false;
        bool mirror_action = true;
        VisualRefereeReact(theRefereePercept.signal, play_sound, mirror_action);
        goto endRefereeChallenge;
      }
      if (state_time > 5000) {
        goto endRefereeChallenge;
      }
    }
  }
  target_state(endRefereeChallenge) {
    action {
      theRefereePercept.runModel = false;
      theRefereePercept.preds.clear();
      theRefereePercept.signal = -1;
    }
  }
}

