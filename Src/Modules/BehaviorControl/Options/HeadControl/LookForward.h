/**
 * @file LookForward
 *
 * Pun intended. Looks straight ahead in a way that the Nao V4's cameras cover the area in front of its feet as well as
 * everything else in front of the robot.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(LookForward) {

  /** Simply sets the necessary angles */
  initial_state(lookForward) {
    action { SetHeadPanTilt(0.f, defaultTilt, fromDegrees(150.f)); }
  }
}
