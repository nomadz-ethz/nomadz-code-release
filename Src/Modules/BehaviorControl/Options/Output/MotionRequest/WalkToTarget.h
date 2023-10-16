/**
 * @file WalkToTarget.h
 *
 *  Sets all members of the MotionRequest representation for executing a TargetMode-WalkRequest
 *  (i.e. Walk to a \c target at a \c speed)
 *  @param speed Walking speeds, in percentage.
 *  @param target Walking target, in mm and radians, relative to the robot.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(WalkToTarget, const Pose2D& speed, const Pose2D& target) {
  /** Set the motion request. */
  initial_state(setRequest) {
    transition {
      if (theMotionInfo.motionRequest.motion == MotionRequest::walk) {
        goto requestIsExecuted;
      }
    }
    action {
      // slow down if there is a player in the view
      auto nearest = libCodeRelease.nearestPlayerInSight(0.8, 250);
      auto finalSpeed = speed;
      if (nearest < 600) {
        finalSpeed.translation.x = std::min(finalSpeed.translation.x, 0.25f);
      } else if (nearest < 1000) {
        finalSpeed.translation.x = std::min(finalSpeed.translation.x, 0.5f);
      }
      theArmMotionRequest.forceReturnDefault = false;

      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      ASSERT(!isnan(target.translation.x));
      theMotionRequest.walkRequest.target = target;
      theMotionRequest.walkRequest.speed = finalSpeed;
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
      // slow down if there is a player in the view
      auto nearest = libCodeRelease.nearestPlayerInSight(0.8, 250);
      auto finalSpeed = speed;
      if (nearest < 600) {
        finalSpeed.translation.x = std::min(finalSpeed.translation.x, 0.25f);
      } else if (nearest < 1000) {
        finalSpeed.translation.x = std::min(finalSpeed.translation.x, 0.5f);
      }

      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      // theMotionRequest.walkRequest.inWalkKickRequest.kickDirection =
      // theMotionRequest.walkRequest.inWalkKickRequest.kickType = InWalkKickRequest::straightKick;
      theMotionRequest.walkRequest.target = target;
      theMotionRequest.walkRequest.speed = finalSpeed;
      // theMotionRequest.walkRequest.kickType = WalkRequest::nokick;
      theMotionRequest.walkRequest.stepRequest = WalkRequest::none;
    }
  }
}
