/**
 * @file HandlePenaltyState.h
 *
 * Handle penalty state (and the actions to be performed after leaving the penalty state).
 * This option is one level higher than the main game state handling as penalties
 * might occur in most game states.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(HandlePenaltyState) {

  /** By default, the robot is not penalized and plays soccer according to the current game state.
     The chestbutton has to be pushed AND released to manually penalize a robot */
  initial_state(notPenalized) {
    transition {
      if (theRobotInfo.penalty != PENALTY_NONE) {
        goto penalized;
      }
    }
    action { HandleGameState(); }
  }

  /** If the penalty is removed say "not penalized" and continue. */
  state(unPenalized) {
    transition { goto notPenalized; }
    action {
      PlaySound("notPenalized.wav");
      HandleGameState();
    }
  }

  /** In case of any penalty, the robots stands still. */
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
      theHeadControlMode = HeadControl::lookForward;
      wasPenalizedBeforePlay = true;
    }
  }
}
