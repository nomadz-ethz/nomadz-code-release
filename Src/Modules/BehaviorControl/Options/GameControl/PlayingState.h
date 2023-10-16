/**
 * @file PlayingState.h
 *
 * Every player gets the full overview of how many players are playing which role and then
 * asks himself what role he was assigned.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(PlayingState) {
  common_transition {
    if (theBehaviorStatus.roleChanged) {
      goto waitForKickOff;
    }
  }

  initial_state(waitForKickOff) {
    transition {
      if (wasPenalizedBeforePlay || isKickingTeam || !isOffensivePlayer || theGameInfo.ballInPlay) {
        goto checkRole;
      }
    }
    action {
      /* If we don't have kickoff, we are allowed to enter the center circle after 10 sec or after the
       * opponent touched the ball. */
      if (theBehaviorStatus.role == BehaviorStatus::striker) {
        const int secondsRemaining = Range<int>(1, 10).limit(std::round((10000 - state_time) / 1000));
        const std::string soundFile = "number" + std::to_string(secondsRemaining) + ".wav";
        // const std::string soundFile = "number10.wav";
        PlaySound(soundFile);
      }
      theHeadControlMode = HeadControl::lookAtBall;
      Stand();
    }
  }

  state(checkRole) {
    transition {
      switch (theBehaviorStatus.role) {
      case BehaviorStatus::striker:
      case BehaviorStatus::supporter:
      case BehaviorStatus::defender:
      case BehaviorStatus::keeper:
        goto activeRoleBehavior;

      case BehaviorStatus::visualRefereeChallenge:
        goto visualRefereeChallenge;
      case BehaviorStatus::dummy:
        goto dummy;
      case BehaviorStatus::testRole:
        goto testRole;
      }
    }
    action {}
  }

  state(activeRoleBehavior) {
    transition {
      if (theWhistle.whistleDetected == true &&
          abs(theRobotPose.translation.x) < theFieldDimensions.xPosOpponentPenaltyMark &&
          thePersonalData.hasBallLock == false && useRefereeChallenge == true) {
        goto refereeChallenge;
      }
      if (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK) {
        goto penaltyState;
      }
      if (theBehaviorStatus.roleChanged) {
        goto checkRole;
      }
    }
    action { HandleUncertainty(); }
  }

  state(penaltyState) {
    transition {
      if (theGameInfo.setPlay != SET_PLAY_PENALTY_KICK) {
        goto activeRoleBehavior;
      }
      if (theBehaviorStatus.roleChanged) {
        goto checkRole;
      }
    }
    action {
      if (isKickingTeam) {
        if (theBehaviorStatus.role == BehaviorStatus::striker) {
          PenaltyStriker();
        } else {
          StandHigh();
        }
      } else {
        if (theBehaviorStatus.role == BehaviorStatus::keeper) {
          PenaltyKeeper();
        } else {
          StandHigh();
        }
      }
    }
  }

  state(visualRefereeChallenge) {
    transition {
      if (theBehaviorStatus.roleChanged) {
        goto checkRole;
      }
    }
    action { VisualRefereeChallenge(); }
  }

  state(testRole) {
    action { TestRole(); }
  }

  state(dummy) // can be used for any type of testing
  {
    action {
      // this is for random testing purposes...
    }
  }
  state(refereeChallenge) {
    transition {
      if (action_done) {
        goto checkRole;
      }
    }
    action { RefereeChallenge(); }
  }
}