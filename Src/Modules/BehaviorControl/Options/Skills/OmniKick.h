/**
 * @file OmniKick.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(OmniKick, float kickAngle, float kickPower, bool usingDefault = true) {
  initial_state(kick) {
    action {

      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target.translation.x = 0.f;
      theMotionRequest.walkRequest.target.translation.y = 0.f;
      theMotionRequest.walkRequest.target.rotation = 0.f;

      if (usingDefault) {
        theMotionRequest.kickRequest.kickMotionType = KickRequest::omniKickDefault;
      } else {
        theMotionRequest.kickRequest.kickMotionType = KickRequest::omniKickLong;
      }
      theMotionRequest.kickRequest.kickAngle = kickAngle;
      theMotionRequest.kickRequest.kickPower = kickPower;
      theMotionRequest.motion = MotionRequest::kick;
    }
  }
}
