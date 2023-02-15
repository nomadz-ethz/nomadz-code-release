/**
 * @file Soccer.h
 *
 * The root option that controls the behavior before the robot actually starts to play
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

option(Soccer) {
  /** Initially, all robot joints are off until the chest button is pressed. */
  initial_state(playDead) {
    transition {
#ifdef TARGET_SIM
      goto standUp; // Don't wait for the button in SimRobot
#endif

      if (action_done) { // head middle button pressed and released
        goto standUp;

        // Skip playDead state at a restart after a crash
      } else if (Global::getSettings().recover) {
        goto standUp;
      }
    }
    action {
      SpecialAction(SpecialActionRequest::playDead);
      ButtonPressedAndReleased(KeyStates::chest, 1000, 0);
    }
  }

  /** The robot stands up and starts to play when stand was executed. */
  state(standUp) {
    transition {
      if (action_done) {
        goto playSoccer;
      }
    }
    action {
      LookForward();
      Stand();
    }
  }

  /**
   * The main state that triggers the actual soccer behavior.
   * It also checks whether the chest button was pressed.
   */
  state(playSoccer) {
    transition {}
    action {
      HandlePenaltyState();
      HeadControl();
    }
  }
}
