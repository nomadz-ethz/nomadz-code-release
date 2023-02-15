/**
 * @file LookRightZeroTilt.h
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(LookRightZeroTilt) {

  initial_state(lookRightZeroTilt) {
    action { SetHeadPanTilt(fromDegrees(-60.f), 0.0f, fromDegrees(150.f)); }
  }
}