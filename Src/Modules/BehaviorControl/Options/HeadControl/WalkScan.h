/**
 * @file WalkScan.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(WalkScan) {
  const float mediumPan = fromDegrees(50.f); // [deg]

  initial_state(initialize) {
    transition {
      if (action_done && state_time > 200) {
        goto lookLeft;
      }
    }
    action { SetHeadPanTilt(0.f, defaultTilt, scanSpeed); }
  }

  state(lookLeft) {
    transition {
      if (action_done && state_time > 200) {
        goto lookRight;
      }
    }
    action { SetHeadPanTilt(mediumPan, defaultTilt, scanSpeed); }
  }

  state(lookRight) {
    transition {
      if (action_done && state_time > 200) {
        goto lookLeft;
      }
    }
    action { SetHeadPanTilt(-mediumPan, defaultTilt, scanSpeed); }
  }
}
