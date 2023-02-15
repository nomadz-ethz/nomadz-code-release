/**
 * @file PowerKick.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(PowerKick, bool leftFoot) {
  initial_state(kick) {
    action {

      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target.translation.x = 0.f;
      theMotionRequest.walkRequest.target.translation.y = 0.f;
      theMotionRequest.walkRequest.target.rotation = 0.f;

      theMotionRequest.kickRequest.kickMotionType = KickRequest::fastKick;
      theMotionRequest.kickRequest.mirror = leftFoot;
      theMotionRequest.motion = MotionRequest::kick;
    }
  }
}
