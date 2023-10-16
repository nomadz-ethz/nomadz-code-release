/**
 * @file HandleGameState.h
 *
 * Triggers the options for the different game states.
 * This option also invokes the get up behavior after a fall, as this is needed in most game states.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */
option(HandleGameState) {

  isOffensivePlayer =
    (theBehaviorStatus.role == BehaviorStatus::striker) || (theBehaviorStatus.role == BehaviorStatus::supporter);

  /** As game state changes are discrete external events and all states are independent of each other,
      a common transition can be used here. */
  state(checkGameState) {
    transition {
      if (theGameInfo.state == STATE_INITIAL) {
        goto initial;
      } else if (theGameInfo.state == STATE_READY) {
        goto ready;
      } else if (theGameInfo.state == STATE_SET) {
        goto set;
      } else if (theGameInfo.state == STATE_PLAYING) {
        goto playing;
      } else if (theGameInfo.state == STATE_FINISHED) {
        goto finished_ref;
      }
    }
  }
  /** Stand still and wait. */
  initial_state(initial) {
    transition {
      if (theGameInfo.state != STATE_INITIAL) {
        goto checkGameState;
      }
    }
    action { Stand(); }
  }

  /** Walk to kickoff position. */
  state(ready) {
    transition {
      if (theFallDownState.state == FallDownState::onGround) {
        goto getUp;
      }
      if (theGameInfo.state != STATE_READY) {
        goto checkGameState;
      }
    }
    action { ReadyState(); }
  }

  /** Stand and look around. */
  state(set) {
    transition {
      if (theFallDownState.state == FallDownState::onGround) {
        goto getUp;
      }
      if (theGameInfo.state != STATE_SET) {
        goto checkGameState;
      }
    }
    action { SetState(); }
  }

  /** Play soccer! */
  state(playing) {
    transition {
      if (theFallDownState.state == FallDownState::onGround) {
        // Deactivate referee model in case it was running
        theRefereePercept.runModel = false;
        goto getUp;
      }
      // Only change state if referee model is no longer running
      if (theGameInfo.state != STATE_PLAYING && theRefereePercept.runModel == false) {
        goto checkGameState;
      }
    }
    action {
      // OUTPUT_TEXT(theWhistle.whistleDetected);
      PlayingState();
    }
  }

  /** Stand still and wait. If somebody wants to implement cheering moves => This is the place. ;-) */
  state(finished_ref) {
    transition {
      if (useRefereeChallenge == false) {
        goto finished;
      } else if (action_done) {
        goto finished;
      } else if (abs(theRobotPose.translation.x) >= theFieldDimensions.xPosOpponentPenaltyMark) {
        goto finished;
      }
    }
    action {
      if (abs(theRobotPose.translation.x) < theFieldDimensions.xPosOpponentPenaltyMark) {
        RefereeChallenge();
      }
    }
  }

  state(finished) {
    transition {
      if (theGameInfo.state != STATE_FINISHED) {
        goto checkGameState;
      }
    }
    action {
      if (state_time < 1000) {
        Stand();
      } else {
        // Use sitdown keeper as it keeps stiffness preventing the player from falling.
        SpecialAction(SpecialActionRequest::sitDownKeeper);
      }
    }
  }

  /** Get up from the carpet. */
  state(getUp) {
    transition {
      if (action_done) {
        goto checkGameState;
      }
    }
    action { GetUp(); }
  }
}
