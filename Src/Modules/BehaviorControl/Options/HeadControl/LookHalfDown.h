/**
 * @file LookHalfDown.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(LookHalfDown) {
  initial_state(lookHalfDown) {
    action { SetHeadPanTilt(0.f, 0.41f, fromDegrees(150.f)); }
  }
}
