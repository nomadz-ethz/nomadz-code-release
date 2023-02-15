/**
 * @file LookDown.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(LookDown) {
  initial_state(lookDown) {
    action { SetHeadPanTilt(0.f, 0.45f, fromDegrees(150.f)); }
  }
}
