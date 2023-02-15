/**
 * @file  ScanLeftRight.h
 *
 * Look forward, left, right, and repeat.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(ScanLeftRight) {

  bool isAllStatesScanned;
  const float mediumPan = fromDegrees(60.f);
  const float maxPan = fromDegrees(120.f);
  const float speed = fromDegrees(80.f);

  common_transition {
    if (libCodeRelease.timeSinceBallWasSeen() < ballValidTime && theRobotPose.timeSinceLastValid < robotPoseValidTime) {
      goto success;
    }
  }

  initial_state(initialize) {
    transition {
      if (state_time > 100) {
        goto lookForward;
      }
    }
    action { isAllStatesScanned = false; }
  }

  /** Simply sets the necessary angles */
  state(lookForward) {
    transition {
      if (action_done) {
        if (isAllStatesScanned) {
          goto success;
        } else {
          goto lookLeft;
        }
      }
    }
    action { SetHeadPanTilt(0.f, 0.4f, speed); }
  }

  state(lookLeft) {
    transition {
      if (action_done && state_time > 1000 && theMotionInfo.isStanding()) {
        goto lookFullLeft;
      } else if (action_done && state_time > 1000) {
        goto lookForward2;
      }
    }
    action { SetHeadPanTilt(mediumPan, 0.2f, speed); }
  }

  state(lookFullLeft) {
    transition {
      if (action_done && state_time > 1000) {
        goto lookForward2;
      }
    }
    action { SetHeadPanTilt(maxPan, 0.1f, speed); }
  }

  state(lookForward2) {
    transition {
      if (action_done && state_time > 1000) {
        goto lookRight;
      }
    }
    action { SetHeadPanTilt(0.f, 0.4f, speed); }
  }

  state(lookRight) {
    transition {
      if (action_done && state_time > 1000 && theMotionInfo.isStanding()) {
        goto lookFullRight;
      } else if (action_done && state_time > 1000) {
        goto lookForward;
      }
    }
    action { SetHeadPanTilt(-mediumPan, 0.2f, speed); }
  }
  state(lookFullRight) {
    transition {
      if (action_done && state_time > 1000) {
        goto lookForward;
      }
    }
    action {
      isAllStatesScanned = true;
      SetHeadPanTilt(-maxPan, 0.1f, speed);
    }
  }

  /** ball was seen this state can be observed by the higher level state machine using action_done*/
  target_state(success) {}
}
