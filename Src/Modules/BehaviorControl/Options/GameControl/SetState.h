/**
 * @file SetState.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(SetState) {
  initial_state(standby) {
    transition {
      if (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK) {
        goto penaltyStandby;
      }
    }
    action {
      // ScanLeftRight here could lead to bad BallModel because of head movement, could lead robot to think ball is in
      // play when it isn't
      // ScanLeftRight allows us to keep being localized by seeing more of the field
      theHeadControlMode = isOffensivePlayer ? HeadControl::lookAtBall : HeadControl::scanLeftRight;
      StandHigh();
    }
  }

  state(penaltyStandby) {
    transition {
      if (theGameInfo.setPlay != SET_PLAY_PENALTY_KICK) {
        goto standby;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      if (theOwnTeamInfo.teamNumber != theGameInfo.kickingTeam && theBehaviorStatus.role == BehaviorStatus::keeper) {
        SpecialAction(SpecialActionRequest::sitDownKeeper);
      } else {
        StandHigh();
      }
    }
  }
}
