/**
 * @file PlayingState.h
 *
 * Every player gets the full overview of how many players are playing which role and then
 * asks himself what role he was assigned.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(PlayingState) {

  initial_state(play) {
    transition {}
    action {
      switch (theRobotBehaviorConfig.role) {
      case BehaviorStatus::striker:
        goto striker;
      case BehaviorStatus::supporter:
        goto supporter;
      case BehaviorStatus::defender:
        goto defender;
      case BehaviorStatus::keeper:
        goto keeper;

      case BehaviorStatus::goToBallAndKick:
        goto goToBallAndKick;
      case BehaviorStatus::penaltyKeeper:
        goto penaltykeeper;
      case BehaviorStatus::penaltyStriker:
        goto penaltystriker;

      case BehaviorStatus::striker2v2:
        goto striker2v2;
      case BehaviorStatus::defender2v2:
        goto defender2v2;
      case BehaviorStatus::visualRefereeChallenge:
        goto visualRefereeChallenge;
      case BehaviorStatus::dummy:
        goto dummy;
      case BehaviorStatus::testRole:
        goto testRole;
      }
    }
  }

  state(dummy) // can be used for any type of testing
  {
    action {
      // this is for random testing purposes...
    }
  }

  state(keeper) {
    action { Keeper(); }
  }

  state(defender) {
    action {
      updateMyDistanceToBall();
      Defender();
    }
  }

  state(supporter) {
    transition {}
    action {
      updateMyDistanceToBall();
      Supporter();
    }
  }

  state(striker) {
    transition {}
    action {
      updateMyDistanceToBall();
      Striker();
    }
  }

  state(goToBallAndKick) {
    transition {}
    action { goToBallAndKick(); }
  }

  state(penaltykeeper) {
    action { PenaltyKeeper(); }
  }

  state(penaltystriker) {
    action { PenaltyStriker(); }
  }

  state(striker2v2) {
    action {
      updateMyDistanceToBall();
      Striker2v2();
    }
  }

  state(defender2v2) {
    action {
      updateMyDistanceToBall();
      Defender2v2();
    }
  }
  state(visualRefereeChallenge) {
    action { VisualRefereeChallenge(); }
  }

  state(testRole) {
    action { TestRole(); }
  }
}
