/**
 * @file Arm.h
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(Arm,
       ArmMotionRequest::Arm arm,
       ArmMotionRequest::ArmMotionId motion,
       bool fast = false,
       bool autoReverse = false,
       int autoReverseTime = 0) {
  initial_state(setRequest) {
    transition {
      if (theArmMotionEngineOutput.arms[arm].move && theArmMotionEngineOutput.arms[arm].motion == motion) {
        goto requestIsExecuted;
      }
    }
    action {
      theArmMotionRequest.motion[arm] = motion;
      theArmMotionRequest.fast[arm] = fast;
      theArmMotionRequest.autoReverse[arm] = autoReverse;
      theArmMotionRequest.autoReverseTime[arm] = autoReverseTime;
    }
  }

  target_state(requestIsExecuted) {
    transition {
      if (!theArmMotionEngineOutput.arms[arm].move || theArmMotionEngineOutput.arms[arm].motion != motion) {
        goto setRequest;
      }
    }
    action {
      theArmMotionRequest.motion[arm] = motion;
      theArmMotionRequest.fast[arm] = fast;
      theArmMotionRequest.autoReverse[arm] = autoReverse;
      theArmMotionRequest.autoReverseTime[arm] = autoReverseTime;
    }
  }
}
