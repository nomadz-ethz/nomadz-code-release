/**
 * @file SearchAndRelocate.h
 *
 * Replacement for the previous relocation and ball search logic
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(RelocateWhileProtectBall) {
  static Angle globalTargetAngle = 0;
  common_transition {}

  initial_state(scan) {
    transition {
      if (state_time > 800) {
        goto checkPoint;
      }
    }
    action {
      theHeadControlMode = HeadControl::scanLeftRightKeeper;
      Stand();
    }
  }

  state(checkPoint) {
    transition {
      if (state_time > 1800) {
        globalTargetAngle = globalTargetAngle + 1;
        goto scan;
      }
    }
    action { CircleAroundBall(globalTargetAngle); }
  }

  target_state(Relocated) {}

  aborted_state(stillLost) {}
}
