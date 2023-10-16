/**
 * @file  ScanRightLeft.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(ScanRightLeft) {
  const float maxPan = fromDegrees(120.f);

  initial_state(initialize) {
    transition {
      if (action_done && state_time > 200) {
        goto lookFullyRight;
      }
    }
    action { SetHeadPanTilt(0.f, defaultTilt, scanSpeed); }
  }

  state(lookFullyLeft) {
    transition {
      if (action_done && state_time > 200) {
        goto lookForward;
      }
    }
    action { SetHeadPanTilt(maxPan, fromDegrees(15.f), scanSpeed); }
  }

  state(lookForward) {
    transition {
      if (action_done && state_time > 200) {
        goto lookFullyRight;
      }
    }
    action { SetHeadPanTilt(0, defaultTilt, scanSpeed); }
  }

  state(lookFullyRight) {
    transition {
      if (action_done && state_time > 200) {
        goto lookFullyLeft;
      }
    }
    action { SetHeadPanTilt(-maxPan, fromDegrees(15.f), scanSpeed); }
  }
}
