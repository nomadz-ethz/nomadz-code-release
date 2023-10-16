/**
 * @file HandleGamePhase.h
 *
 * Triggers the options for the different game states.
 * This option also invokes the get up behavior after a fall, as this is needed in most game states.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

bool wasPenalizedBeforePlay = false;
bool isOffensivePlayer = false;
bool isKickingTeam = false;

option(HandleGamePhase) {
  isKickingTeam = (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam);

  initial_state(normalPhase) {
    transition {
      if (theGameInfo.gamePhase == GAME_PHASE_TIMEOUT) {
        goto timeoutPhase;
      } else if (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT) {
        goto penaltyShootout;
      }
    }
    action { HandleGameState(); }
  }

  /** Stand still and wait. If somebody wants to implement cheering moves => This is the place. ;-) */
  state(timeoutPhase) {
    transition {
      if (theGameInfo.gamePhase != GAME_PHASE_TIMEOUT) {
        goto normalPhase;
      }
    }
    action {
      theBehaviorLEDRequest.leftEyeColor = BehaviorLEDRequest::green;
      theBehaviorLEDRequest.rightEyeColor = BehaviorLEDRequest::green;
      StandHigh();
    }
  }

  /** Get up from the carpet. */
  state(penaltyShootout) {
    transition {
      if (theFallDownState.state == FallDownState::onGround) {
        goto getUp;
      }
      if (theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT) {
        goto normalPhase;
      }
    }
    action {
      if (theGameInfo.state == STATE_PLAYING) {
        if (isKickingTeam) {
          PenaltyStriker();
        } else {
          PenaltyKeeper();
        }
      } else {
        StandHigh();
      }
    }
  }

  state(getUp) {
    transition {
      if (action_done) {
        goto normalPhase;
      }
    }
    action { GetUp(); }
  }
}