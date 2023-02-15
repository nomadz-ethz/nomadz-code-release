/**
 * @file LookLeftZeroTilt.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(LookLeftZeroTilt) {

  initial_state(looLeftZeroTilt) {
    action { SetHeadPanTilt(fromDegrees(60.f), 0.0f, fromDegrees(150.f)); }
  }
}