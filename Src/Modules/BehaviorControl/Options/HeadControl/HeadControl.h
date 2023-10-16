/**
 * @file HeadControl.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(HeadControl) {
  common_transition {
    // if(theGameInfo.state != STATE_INITIAL)
    //   goto off;
    // theHeadControlMode = static_cast<HeadControl::Mode>(theBehaviorStatus.headMode);

    switch (theHeadControlMode) {
    case HeadControl::off:
      goto off;
    case HeadControl::lookForward:
      goto lookForward;
    case HeadControl::lookAtBall:
      goto lookAtBall;
    case HeadControl::lookAtTarget:
      goto lookAtTarget;
    case HeadControl::lookAtTargetWithTilt:
      goto lookAtTargetWithTilt;
    case HeadControl::scanLeftRight:
      goto scanLeftRight;
    case HeadControl::walkScan:
      goto walkScan;
    default:
      goto none;
    }
  }

  initial_state(none) {}
  state(off) { action SetHeadPanTilt(JointData::off, JointData::off, 0.f); }
  state(lookForward) { action LookForward(); }
  state(lookAtTarget) { action LookAtTarget(lookTargetAngle); }
  state(lookAtTargetWithTilt) { action LookAtTargetWithTilt(lookTargetAngle, lookTargetTilt); }
  state(lookAtBall) { action LookAtBall(); }
  state(scanLeftRight) { action ScanLeftRight(); }
  state(walkScan) { action WalkScan(); }
}

struct HeadControl {
  ENUM(Mode, none, off, lookForward, lookAtBall, lookAtTarget, scanLeftRight, walkScan, lookAtTargetWithTilt)
};

HeadControl::Mode theHeadControlMode; /**< The head control mode executed by the option HeadControl. */
float lookTargetTilt;
float lookTargetAngle;
float scanSpeed = fromDegrees(80.f);     // [deg/s]
float defaultSpeed = fromDegrees(200.f); // [deg/s]
float defaultTilt = fromDegrees(21.f);   // [rad]
