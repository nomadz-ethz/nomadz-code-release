/**
 * @file LookForFeatures.h
 *
 * Maximize time when features (center circle, penalty area) are seen, based on
 * information from RobotPose.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(LookForFeatures) {

  initial_state(lookForward) {
    transition {
      if (theFrameInfo.getTimeSince(theRobotPose.timeLastSensorUpdate) > 5000) {
        if (rand() % 2 == 0) {
          goto scanLeft;
        } else {
          goto scanRight;
        }
      }
    }
    action { LookForward(); }
  }

  state(scanLeft) {
    transition {
      if (theFrameInfo.getTimeSince(theRobotPose.timeLastSensorUpdate) < 5000) {
        goto lookForward;
      }
      if (theJointData.angles[JointData::HeadYaw] > fromDegrees(50.f)) {
        goto scanRight;
      }
    }
    action {
      SetHeadPanTilt(fromDegrees(55.f), 0.32f, fromDegrees(35.f)); // TODO Find better pan angle
    }
  }

  state(scanRight) {
    transition {
      if (theFrameInfo.getTimeSince(theRobotPose.timeLastSensorUpdate) < 5000) {
        goto lookForward;
      }
      if (theJointData.angles[JointData::HeadYaw] < fromDegrees(-50.f)) {
        goto scanLeft;
      }
    }
    action {
      SetHeadPanTilt(fromDegrees(-55.f), 0.32f, fromDegrees(35.f)); // TODO Find better pan angle
    }
  }
}
