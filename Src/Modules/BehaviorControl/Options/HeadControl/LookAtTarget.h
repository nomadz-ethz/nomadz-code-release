/**
 * @file LookAtTarget.h
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(LookAtTarget, float lookTargetAngle) {
  initial_state(look) {
    transition {}
    action {
      // Robot looks up for some reason when value here is higher.
      const float pan = std::max(-0.33f, std::min(lookTargetAngle, 0.33f));
      SetHeadPanTilt(pan, 0.38f, fromDegrees(120.f));
    }
  }
}
