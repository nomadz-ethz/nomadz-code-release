/**
 * @file WalkScan.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(WalkScan) {

  static bool lookedLeft = false, lookedRight = false;

  initial_state(initialize) {
    transition {
      if (state_time > 100) {
        goto lookLeft;
      }
    }
    action {}
  }

  /** Simply sets the necessary angles */
  state(lookForward) {
    transition {
      if (action_done && state_time > 1400) {
        if (lookedLeft == false && lookedRight == true) {
          goto lookLeft;
        } else {
          goto lookRight;
        }
      }
    }
    action { SetHeadPanTilt(0.f, 0.32f, fromDegrees(25.f)); }
  }

  state(lookLeft) {
    transition {
      if (action_done && state_time > 1400) {
        goto lookForward;
      }
    }
    action {
      SetHeadPanTilt(0.3f, 0.32f, fromDegrees(25.f));
      lookedLeft = true;
      lookedRight = false;
    }
  }

  state(lookRight) {
    transition {
      if (action_done && state_time > 1400) {
        goto lookForward;
      }
    }
    action {
      SetHeadPanTilt(-0.3f, 0.32f, fromDegrees(25.f));
      lookedLeft = false;
      lookedRight = true;
    }
  }
}
