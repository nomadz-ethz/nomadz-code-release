/**
 * @file Relocate.h
 *
 * This option provides the needed behavior for relocalization
 * with the LocalizationProvider (SLAM-like)
 *                      -----------
 *       target_state: | relocated |  <-  (bestCertainty > thresholdCertainty)
 *                      -----------
 *  -------        ----------        -----------        ----------
 * | start |  ->  | lookLeft |  ->  | lookRight |  ->  | reorient |  _
 *  -------        ----------        -----------        ----------    |
 *      /|\        actually this diagram is not valid anymore         |
 *       |____________________________________________________________|    // outdated
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(Relocate) {

  common_transition {
    if (theRobotPose.timeSinceLastValid < 5000.f) {
      goto success;
    }
  }

  // Initial state just to play this sound on entry
  initial_state(enter) {
    transition { goto lookAheadDown; }
    action {
      theHeadControlMode = HeadControl::lookForward;
      PlaySound("lost.wav");
    }
  }

  state(lookAheadDown) {
    transition {
      if (state_time > 1000 + 500) {
        goto lookRightDown;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }

  state(lookLeftDown) {
    transition {
      if (state_time > 1000 + 400 + 500) {
        goto lookRightDown;
      }
    }
    action {
      theHeadControlMode = HeadControl::none;
      SetHeadPanTilt(fromDegrees(60.f), 0.32f, fromDegrees(150.f));
      Stand();
    }
  }

  state(lookRightDown) {
    transition {
      if (state_time > 1000 + 800 + 500) {
        goto reorient;
      }
    }
    action {
      theHeadControlMode = HeadControl::none;
      SetHeadPanTilt(fromDegrees(-60.f), 0.32f, fromDegrees(150.f));
      Stand();
    }
  }

  state(reorient) {
    transition {
      if (state_time > 2000 + 1000) {
        goto lookAheadDown;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      WalkAtSpeed(Pose2D(1.f, 0.f, 0.f));
    }
  }

  state(verifySuccess) {
    transition {
      if (state_time > 4000) {
        goto lookAheadDown;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }

  target_state(success) {
    action {
      OUTPUT_TEXT("Relocate: success");

      theHeadControlMode = HeadControl::lookForward;
      PlaySound("allright.wav");
      Stand();
    }
  }
}
