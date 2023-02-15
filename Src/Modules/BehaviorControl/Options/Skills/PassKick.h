/**
 * @file PassKick.h
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(PassKick, bool leftFoot) {
  initial_state(kick) {
    action {

      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target.translation.x = 0.f;
      theMotionRequest.walkRequest.target.translation.y = 0.f;
      theMotionRequest.walkRequest.target.rotation = 0.f;

      theMotionRequest.kickRequest.kickMotionType = KickRequest::fastKickSlower;
      theMotionRequest.kickRequest.mirror = false; // leftFoot;
      theMotionRequest.motion = MotionRequest::kick;
    }
  }
}
