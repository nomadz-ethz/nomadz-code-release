/**
 * @file ReadyAndLocate.h
 *
 * This is a faster and more flexible way for the robots to locate when entering the pitch in a ready-state.
 * It will not start its motion until it has seen both goals for at least a x-amount of time or it has turned
 * its head to the maximum amound instead. The algorithm is not looped since we do not assume that the robot
 * will observe the goal the second time it turns its head when it didn't observe it the first time.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

bool readyAndLocated = false;
float headRotation;

option(ReadyAndLocate) {
  initial_state(ready) {
    transition { goto lookLeft; }
    action {}
  }

  state(lookLeft) {
    transition {                                              // Use headRotation as a time stamp as well
      if (((state_time - headRotation * 60) > 6000)           // Goal must be observed for at least 4 sec. to locate properly
          || (theHeadMotionRequest.pan > fromDegrees(115.f))) // If the head rotation is maximum, go to the next state.
      {
        goto lookRight;
      }
    }
    action {
      if (!((theFrameInfo.getTimeSince(theGoalPercept.timeWhenCompleteGoalLastSeen) < 1000) &&
            (theHeadMotionRequest.pan > fromDegrees(20)))) // Make sure the goal was on your left side.
      {
        headRotation = state_time / 60;
      }
      SetHeadPanTilt(fromDegrees(headRotation), 0.20f, fromDegrees(150.f));
    }
  }

  state(lookRight) {
    transition { // Head angle at maximum after 6 seconds.
      if ((state_time + headRotation * 60) > 6000 || theHeadMotionRequest.pan < fromDegrees(-115.f)) {
        readyAndLocated = true;
      }
    }
    action {
      if (!((theFrameInfo.getTimeSince(theGoalPercept.timeWhenCompleteGoalLastSeen) < 1000) &&
            (theHeadMotionRequest.pan < fromDegrees(-20)))) // Make sure the goal was on your left side.
      {
        headRotation = -state_time / 60;
      }
      SetHeadPanTilt(fromDegrees(headRotation), 0.20f, fromDegrees(150.f));
    }
  }
}
