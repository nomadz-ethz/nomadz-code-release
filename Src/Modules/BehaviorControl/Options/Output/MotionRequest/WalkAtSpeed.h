/**
 * @file WalkAtSpeed.h
 *
 * Sets all members of the MotionRequest representation for executing a SpeedMode-WalkRequest
 *  (i.e. Walk at a \c speed)
 *  @param speed Walking speeds, in mm/s and radian/s.
 *                e.g.  Pose2D(0.f, 50.f, 0.f) lets move the robot forward at 50 mm/s
 *                      Pose2D(0.f, 50.f, 50.f) lets move the robot diagonal
 *                      Pose2D(0.2f, 100.f, 0.f) lets move the robot in a circle
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(WalkAtSpeed, const Pose2D& speed) {
  /** Set the motion request. */
  initial_state(setRequest) {
    transition {
      if (theMotionInfo.motionRequest.motion == MotionRequest::walk) {
        goto requestIsExecuted;
      }
    }
    action {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::speedMode;
      theMotionRequest.walkRequest.speed = speed;
      // theMotionRequest.walkRequest.kickType = WalkRequest::nokick;
      theMotionRequest.walkRequest.stepRequest = WalkRequest::none;
    }
  }

  /** The motion process has started executing the request. */
  target_state(requestIsExecuted) {
    transition {
      if (theMotionInfo.motionRequest.motion != MotionRequest::walk) {
        goto setRequest;
      }
    }
    action {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::speedMode;
      theMotionRequest.walkRequest.speed = speed;
      // theMotionRequest.walkRequest.kickType = WalkRequest::nokick;
      theMotionRequest.walkRequest.stepRequest = WalkRequest::none;
    }
  }
}
