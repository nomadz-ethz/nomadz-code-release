/**
 * @file ReadyState.h
 *
 * Assigns roles to the players. The robotnumbers can be set in the teams.cfg file.
 * The players have to have been in readystate before they will perform the actions in the playing
 * state.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(ReadyState) {
  const float posTol = 100.f;
  const float angleTol = fromDegrees(10.f);

  Pose2D startPose;

  common_transition {
    if (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK) {
      switch (theRobotBehaviorConfig.role) {
      case BehaviorStatus::keeper:
        if (theOwnTeamInfo.teamNumber != theGameInfo.kickingTeam) {
          startPose = Pose2D(0.f, setPosPenaltyKeeper);
        } else {
          startPose = Pose2D(0.f, setPosKeeper);
        }
        break;
      case BehaviorStatus::striker:
        if (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam) {
          startPose = Pose2D(0.f, setPosPenaltyStriker);
        } else {
          startPose = Pose2D(3.14f, setPosStriker);
        }
        break;
      case BehaviorStatus::supporter:
        if (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam) {
          if (theTeamMateData.isPenalized[5]) {
            startPose = Pose2D(0.f, setPosPenaltyStriker);
          } else {
            startPose = Pose2D(0.f, setPosPenaltySupporter);
          }
        } else {
          startPose = Pose2D(3.14f, setPosSupporter);
        }
        break;
      case BehaviorStatus::defender:
        if (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam) {
          startPose = Pose2D(0.f, (theRobotInfo.number % 2 == 0) ? setPosPenaltyDefenderEven : setPosPenaltyDefenderOdd);
        } else {
          startPose = Pose2D(3.14f, (theRobotInfo.number % 2 == 0) ? setPosPenaltyDefenderEven : setPosPenaltyDefenderOdd);
        }
        break;
      default:
        startPose = Pose2D(0.f, setPosKickoffOther);
        break;
      }
    } else {
      if (theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam) {
        switch (theRobotBehaviorConfig.role) {
        case BehaviorStatus::keeper:
          startPose = Pose2D(0.f, setPosKickoffKeeper);
          break;
        case BehaviorStatus::striker:
          startPose = Pose2D(fromDegrees(25.f), setPosKickoffStriker);
          break;
        case BehaviorStatus::supporter:
          if (theTeamMateData.isPenalized[5]) {
            startPose = Pose2D(fromDegrees(25.f), setPosKickoffStriker);
          } else {
            startPose = Pose2D(0.f, setPosKickoffSupporter);
          }
          break;
        case BehaviorStatus::defender:
          startPose = Pose2D(0.f, (theRobotInfo.number % 2 == 0) ? setPosKickoffDefenderEven : setPosKickoffDefenderOdd);
          break;
        default:
          startPose = Pose2D(0.f, setPosKickoffOther);
          break;
        }
      } else {
        switch (theRobotBehaviorConfig.role) {
        case BehaviorStatus::keeper:
          startPose = Pose2D(0.f, setPosKeeper);
          break;
        case BehaviorStatus::striker:
          startPose = Pose2D(0.f, setPosStriker);
          break;
        case BehaviorStatus::supporter:
          if (theTeamMateData.isPenalized[5]) {
            startPose = Pose2D(0.f, setPosStriker);
          } else {
            startPose = Pose2D(0.f, setPosSupporter);
          }
          break;
        case BehaviorStatus::defender:
          startPose = Pose2D(0.f, (theRobotInfo.number % 2 == 0) ? setPosDefenderEven : setPosDefenderOdd);
          break;
        default:
          startPose = Pose2D(0.f, setPosOther);
          break;
        }
      }
    }
  }

  initial_state(initial) {
    transition { goto walkToStartPose; }
    action {}
  }

  state(walkToStartPose) {
    transition {
      if (((startPose.translation - theRobotPose.translation).abs() < posTol) &&
          (std::abs(startPose.rotation - theRobotPose.rotation) < angleTol)) {
        goto ready;
      }
    }

    action {
      switch (theRobotBehaviorConfig.role) {
      case BehaviorStatus::striker:
      case BehaviorStatus::supporter:
      case BehaviorStatus::defender:
      case BehaviorStatus::keeper:
        theHeadControlMode = HeadControl::walkScan;
        TravelTo(startPose);
        break;

      case BehaviorStatus::penaltyKeeper:
      case BehaviorStatus::penaltyStriker:
        Stand();
        break;

      case BehaviorStatus::dummy:
        Stand();
        break;
      case BehaviorStatus::testRole:
        Stand();
        break;
      case BehaviorStatus::dropIn:
        Stand();
        break;
      }
    }
  }

  state(ready) { // When the robot is at its destination pose, keep looking around to gather as
    // much information as possible.
    transition { // If it realizes that it's in the wrong position (quite likely!), it should reposition itself.
      if (((startPose.translation - theRobotPose.translation).abs() > 2.f * posTol) ||
          (std::abs(startPose.rotation - theRobotPose.rotation) > 2.f * angleTol)) {
        goto initial;
      }
    }
    action {
      Stand();
      theHeadControlMode = HeadControl::scanLeftRight;
    }
  }
}
