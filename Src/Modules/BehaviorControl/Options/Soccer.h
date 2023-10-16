/**
 * @file Soccer.h
 *
 * The root option that controls the behavior before the robot actually starts to play
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
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
      if (theRobotInfo.penalty != PENALTY_NONE) {
        goto penalized;
      }
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
    transition {
      if (theRobotInfo.penalty != PENALTY_NONE) {
        goto penalized;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      theBehaviorLEDRequest.leftEyeColor = BehaviorLEDRequest::defaultColor;
      theBehaviorLEDRequest.rightEyeColor = BehaviorLEDRequest::defaultColor;
      if (theGameInfo.competitionType == COMPETITION_TYPE_NORMAL) {
        // Normal Competition behavior
        HandleGamePhase();
        HeadControl();

      } else {
        // Other Competition behavior. Currently not implemented (e.g. Dynamic Ball Handling )
        theBehaviorLEDRequest.leftEyeColor = BehaviorLEDRequest::red;
        theBehaviorLEDRequest.rightEyeColor = BehaviorLEDRequest::red;
        StandHigh();
      }
    }
  }

  state(penalized) {
    transition {
      if (theRobotInfo.penalty == PENALTY_NONE) {
        goto unPenalized;
      }
    }
    action {
      if (0 < theRobotInfo.secsTillUnpenalised && theRobotInfo.secsTillUnpenalised <= 10) {
        PlaySound("number" + std::to_string(theRobotInfo.secsTillUnpenalised) + ".wav");
      } else {
        PlaySound("penalized.wav");
      }
      StandHigh();
      wasPenalizedBeforePlay = true;

      LookForward();
    }
  }

  state(unPenalized) {
    transition {
      if (action_done) {
        goto playSoccer;
      }
    }
    action { PlaySound("notPenalized.wav"); }
  }
}
