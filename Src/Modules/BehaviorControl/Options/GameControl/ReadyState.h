/**
 * @file ReadyState.h
 *
 * Assigns roles to the players. The robotnumbers can be set in the teams.cfg file.
 * The players have to have been in readystate before they will perform the actions in the playing
 * state.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(ReadyState) {
  const float posTol = 100.f;
  const float angleTol = fromDegrees(10.f);
  Pose2D startPose = theFieldPosition.readyPose;

  initial_state(initial) {
    transition {
      if (state_time > 2000) {
        goto walkToStartPose;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      Stand();
    }
  }

  state(walkToStartPose) {
    transition {
      if (state_time > 10000 && theRobotPose.lost) {
        goto relocate;
      }
      if (((startPose.translation - theRobotPose.translation).abs() < posTol) &&
          (std::abs(startPose.rotation - theRobotPose.rotation) < angleTol)) {
        goto ready;
      }
    }

    action {
      switch (theBehaviorStatus.role) {
      case BehaviorStatus::striker:
      case BehaviorStatus::supporter:
      case BehaviorStatus::defender:
      case BehaviorStatus::keeper:
        theHeadControlMode = HeadControl::walkScan;
        TravelTo(startPose);
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
      theHeadControlMode = HeadControl::lookAtBall;
    }
  }

  state(relocate) {
    transition {
      if (action_done || state_time > 10000) {
        goto walkToStartPose;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      SearchAndRelocate(true);
    }
  }
}
