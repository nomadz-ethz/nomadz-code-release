/**
 * @file WalkAtSpeedPercentage.h
 *
 * Sets all members of the MotionRequest representation for executing a PercentageSpeedMode-WalkRequest
 *  (i.e. Walk at a \c speed)
 *  @param speed Walking speeds, in percentage.
 *                e.g.  Pose2D(0.f, 1.f, 0.f) lets move the robot forward at full speed
 *                      Pose2D(0.f, 0.5f, 0.5f) lets move the robot diagonal at half of the possible speed
 *                      Pose2D(0.5f, 1.f, 0.f) lets move the robot in a circle
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(WalkAtSpeedPercentage, const Pose2D& speed) {
  /** Set the motion request. */
  initial_state(setRequest) {
    transition {
      if (theMotionInfo.motionRequest.motion == MotionRequest::walk) {
        goto requestIsExecuted;
      }
    }
    action {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::percentageSpeedMode;
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
      theMotionRequest.walkRequest.mode = WalkRequest::percentageSpeedMode;
      theMotionRequest.walkRequest.speed = speed;
      // theMotionRequest.walkRequest.kickType = WalkRequest::nokick;
      theMotionRequest.walkRequest.stepRequest = WalkRequest::none;
    }
  }
}
