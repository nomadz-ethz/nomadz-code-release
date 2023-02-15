/**
 * @file RelocateWhileSearchingBall.h
 *
 * Perform Motion which helps the robot to identify both its position and the balls position.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(RelocateWhileSearchingBall) {

  common_transition {}

  initial_state(scan) {
    transition {
      if (state_time > 1000) {
        goto rotate;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      Stand();
    }
  }
  state(rotate) {
    transition {
      if (action_done) {
        goto scan;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2D(1.f, 0.f, 0.f), Pose2D(1.f, 0.f, 0.f));
    }
  }
  target_state(success) {}

  aborted_state(failure) {}
}

// TODO Remove superfluous function
// Vector2<> generateTargetNearCenter() {
//    const bool side = (rand() % 2 == 1);
//    const float centerRadius = theFieldDimensions.centerCircleRadius;
//    return gloToRel(Vector2<>(0.f, side ? (-centerRadius - rand() % 1000) : (centerRadius + rand() % 1000)));
//}
