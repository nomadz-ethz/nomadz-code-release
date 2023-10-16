/**
 * @file PowerKick.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(PowerKick) {
  initial_state(kick) {
    action {
      theArmMotionRequest.forceReturnDefault = true;
      theMotionRequest.kickRequest.kickMotionType = KickRequest::fastKick;
      theMotionRequest.motion = MotionRequest::kick;
    }
  }
}
