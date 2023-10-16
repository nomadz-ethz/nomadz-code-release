/**
 * @file LookAtTargetWithTilt.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(LookAtTargetWithTilt, float lookTargetAngle, float lookTargetTilt) {
  initial_state(look) {
    transition {}
    action {
      // Robot looks up for some reason when value here is higher.
      const float pan = lookTargetAngle; // std::max(-0.33f, std::min(lookTargetAngle, 0.33f));
      const float tilt = lookTargetTilt;
      SetHeadPanTilt(pan, tilt, defaultSpeed);
    }
  }
}
