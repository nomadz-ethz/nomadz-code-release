/**
 * @file RelocateWhileProtectBall.h
 *
 * Replacement for the previous relocation and ball search logic
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(RelocateWhileProtectBall) {
  static Angle globalTargetAngle = 0;
  common_transition {}

  initial_state(scan) {
    transition {
      if (state_time > 1500) {
        goto checkPoint;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      Stand();
    }
  }

  state(checkPoint) {
    transition {
      if (action_done || state_time > 5000) {
        globalTargetAngle = globalTargetAngle + 1;
        goto scan;
      }
    }
    action { PatternCircleAround(globalTargetAngle); }
  }

  target_state(Relocated) {}

  aborted_state(stillLost) {}
}
