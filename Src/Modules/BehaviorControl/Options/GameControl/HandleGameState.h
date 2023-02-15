/**
 * @file HandleGameState.h
 *
 * Triggers the options for the different game states.
 * This option also invokes the get up behavior after a fall, as this is needed in most game states.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

bool wasPenalizedBeforePlay = false;
bool initial_pass;

option(HandleGameState) {

  const bool isOffensivePlayer =
    (theRobotBehaviorConfig.role == BehaviorStatus::striker) || (theRobotBehaviorConfig.role == BehaviorStatus::supporter);
  const bool hasKickOff = (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam);

  /** As game state changes are discrete external events and all states are independent of each other,
      a common transition can be used here. */
  common_transition {
    if (theGameInfo.competitionType == COMPETITION_TYPE_1VS1_CHALLENGE) {
      goto onevsone;
    } else if (theGameInfo.competitionType == COMPETITION_TYPE_NORMAL ||
               theGameInfo.competitionType == COMPETITION_TYPE_CHALLENGE_SHIELD) {
      if (theGameInfo.gamePhase == GAME_PHASE_NORMAL || theGameInfo.gamePhase == GAME_PHASE_OVERTIME) {
        if (theGameInfo.state == STATE_INITIAL) {
          goto initial;
        } else if (theGameInfo.state == STATE_FINISHED) {
          goto finished;
        } else if (theFallDownState.state == FallDownState::onGround) {
          goto getUp;
        } else if (theGameInfo.state == STATE_READY) {
          goto ready;
        } else if (theGameInfo.state == STATE_SET) {
          goto set;
        } else if (theGameInfo.state == STATE_PLAYING) {
          goto playing;
        }
      } else if (theGameInfo.gamePhase == GAME_PHASE_TIMEOUT) {
        goto timeout;
      } else if (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT) {
        if (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam && theFallDownState.state == FallDownState::onGround) {
          goto getUp;
        } else {
          // All robots that are not playing should have been disabled.
          if (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam) {
            goto penaltystriker;
          } else {
            goto penaltykeeper;
          }
        }
      } else {
        goto error;
      }
    } else {
      goto error;
    }
  }

  /** Stand still and wait. */
  initial_state(initial) {
    action {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }

  /** Stand still and wait. If somebody wants to implement cheering moves => This is the place. ;-) */
  state(finished) {
    action {
      theHeadControlMode = HeadControl::lookForward;
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
    action { GetUp(); }
  }

  /** Walk to kickoff position. */
  state(ready) {
    action {
      theHeadControlMode = HeadControl::lookForward;
      initial_pass = true;
      ReadyState();
    }
  }

  /** Stand and look around. */
  state(set) {
    action {
      if (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK) {
        theHeadControlMode = HeadControl::lookAtBall;
        if (theOwnTeamInfo.teamNumber != theGameInfo.kickingTeam && theRobotBehaviorConfig.role == BehaviorStatus::keeper) {
          SpecialAction(SpecialActionRequest::sitDownKeeper);
        } else {
          StandHigh();
        }
      } else {
        if (isOffensivePlayer) {
          // ScanLeftRight here could lead to bad BallModel because of head movement, could lead robot to think ball is in
          // play when it isn't
          theHeadControlMode = HeadControl::lookAtBall;
        } else {
          // ScanLeftRight allows us to keep being localized by seeing more of the field
          theHeadControlMode = HeadControl::scanLeftRight;
        }
        StandHigh();
      }
      wasPenalizedBeforePlay = false;
      initial_pass = true;
    }
  }

  /** Play soccer! */
  state(playing) {
    action {
      if (!wasPenalizedBeforePlay) // We do not want to wait if we return from a penalty.
      {
        /* If we don't have kickoff, we are allowed to enter the center circle after 10 sec or after the
         * opponent touched the ball. */
        if (!hasKickOff && isOffensivePlayer) {
          if (theGameInfo.ballInPlay || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT) {
            PlayingState();
          } else {
            int countdownSpeaker = 4;
            if (theTeamMateData.isFullyActive[5]) {
              countdownSpeaker = 5;
            }
            const bool sayCountdown = (countdownSpeaker == theRobotInfo.number);
            if (sayCountdown) {
              const int secondsRemaining = Range<int>(1, 10).limit(std::round((10000 - state_time) / 1000));
              const std::string soundFile = "number" + std::to_string(secondsRemaining) + ".wav";
              // const std::string soundFile = "number10.wav";
              PlaySound(soundFile);
            }
            theHeadControlMode = HeadControl::lookAtBall;
            Stand();
          }
        } else {
          PlayingState();
        }
      } else {
        PlayingState();
      }
    }
  }

  state(error) {
    action {
      theBehaviorLEDRequest.leftEyeColor = BehaviorLEDRequest::red;
      theBehaviorLEDRequest.rightEyeColor = BehaviorLEDRequest::red;
      StandHigh();
    }
  }

  state(timeout) {
    action {
      theBehaviorLEDRequest.leftEyeColor = BehaviorLEDRequest::green;
      theBehaviorLEDRequest.rightEyeColor = BehaviorLEDRequest::green;
      StandHigh();
    }
  }

  state(penaltykeeper) {
    action {
      if (theGameInfo.state == STATE_PLAYING) {
        PenaltyKeeper();
      } else {
        StandHigh();
      }
    }
  }
  state(penaltystriker) {
    if (theGameInfo.state == STATE_PLAYING) {
      PenaltyStriker();
    } else {
      StandHigh();
    }
  }

  state(onevsone) { OneVsOneGame(); }
}
