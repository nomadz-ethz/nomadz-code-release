/**
 * @file LookForwardZeroTilt.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(LookForwardZeroTilt) {

  initial_state(lookForwardZeroTilt) {
    action { SetHeadPanTilt(0.0f, 0.0f, fromDegrees(150.f)); }
  }
}