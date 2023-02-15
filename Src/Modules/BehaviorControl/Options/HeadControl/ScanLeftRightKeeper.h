/**
 * @file ScanLeftRightKeeper.h
 *
 * This is a shorter version of the ScanLeftRight search for ball head movement
 * as the Keeper must neither rotate nor look over his shoulder. The ball is
 * always assumed to be in front of him.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

/** Look forward, left, right, and repeat.*/
option(ScanLeftRightKeeper) {

  const float mediumPan = fromDegrees(50.f); // [deg]
  const float maxPan = fromDegrees(120.f);   // [deg]

  initial_state(initialize) {
    transition {
      if (state_time > 100) {
        goto lookForward;
      }
    }
    action {}
  }

  /** Simply sets the necessary angles */
  state(lookForward) {
    transition {
      if (action_done && state_time > 1200 + 1200) {
        goto lookLeft;
      }
    }
    action { SetHeadPanTilt(0.f, 0.4f, fromDegrees(100.f)); }
  }

  state(lookLeft) {
    transition {
      if (action_done && state_time > 1200 + 500) {
        goto lookFullLeft;
      }
    }
    action { SetHeadPanTilt(mediumPan, 0.0f, fromDegrees(100.f)); }
  }

  state(lookFullLeft) {
    transition {
      if (action_done && state_time > 1200 + 700) {
        goto lookForwardIntermediate;
      }
    }
    action { SetHeadPanTilt(maxPan, 0.1f, fromDegrees(100.f)); }
  }

  state(lookForwardIntermediate) {
    transition {
      if (action_done && state_time > 1200 + 1200) {
        goto lookRight;
      }
    }
    action { SetHeadPanTilt(0.f, 0.3f, fromDegrees(100.f)); }
  }

  state(lookRight) {
    transition {
      if (action_done && state_time > 1200 + 500) {
        goto lookFullRight;
      }
    }
    action { SetHeadPanTilt(-mediumPan, 0.0f, fromDegrees(100.f)); }
  }
  state(lookFullRight) {
    transition {
      if (action_done && state_time > 1200 + 700) {
        goto lookForward;
      }
    }
    action { SetHeadPanTilt(-maxPan, 0.1f, fromDegrees(100.f)); }
  }
}
