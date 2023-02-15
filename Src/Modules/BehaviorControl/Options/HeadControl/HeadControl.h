/**
 * @file HeadControl.h
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(HeadControl) {
  common_transition {
    // if(theGameInfo.state != STATE_INITIAL)
    //   goto off;

    switch (theHeadControlMode) {
    case HeadControl::off:
      goto off;
    case HeadControl::lookForward:
      goto lookForward;
    case HeadControl::lookDown:
      goto lookDown;
    case HeadControl::lookHalfDown:
      goto lookHalfDown;
    case HeadControl::lookAtBall:
      goto lookAtBall;
    case HeadControl::lookBallGoal:
      goto lookBallGoal;
    case HeadControl::readyAndLocate:
      goto readyAndLocate;
    case HeadControl::lookAtTarget:
      goto lookAtTarget;
    case HeadControl::lookForwardZeroTilt:
      goto lookForwardZeroTilt;
    case HeadControl::lookLeftZeroTilt:
      goto lookLeftZeroTilt;
    case HeadControl::lookRightZeroTilt:
      goto lookRightZeroTilt;
    case HeadControl::scanLeftRight:
      goto scanLeftRight;
    case HeadControl::scanLeftRightKeeper:
      goto scanLeftRightKeeper;
    case HeadControl::lookForFeatures:
      goto lookForFeatures;
    case HeadControl::walkScan:
      goto walkScan;
    default:
      goto none;
    }
  }

  initial_state(none) {}
  state(off) { action SetHeadPanTilt(JointData::off, JointData::off, 0.f); }
  state(lookForward) { action LookForward(); }
  state(lookDown) { action LookDown(); }
  state(lookHalfDown) { action LookHalfDown(); }
  state(readyAndLocate) { action ReadyAndLocate(); }
  state(lookAtTarget) { action LookAtTarget(lookTargetAngle); }
  state(lookAtBall) { action LookAtBall(); }
  state(lookBallGoal) { action LookBallGoal(); }
  state(lookForwardZeroTilt) { action LookForwardZeroTilt(); }
  state(lookLeftZeroTilt) { action LookLeftZeroTilt(); }
  state(lookRightZeroTilt) { action LookRightZeroTilt(); }
  state(scanLeftRight) { action ScanLeftRight(); }
  state(scanLeftRightKeeper) { action ScanLeftRightKeeper(); }
  state(lookForFeatures) { action LookForFeatures(); }
  state(walkScan) { action WalkScan(); }
}

struct HeadControl {
  ENUM(Mode,
       none,
       off,
       lookForward,
       lookDown,
       lookHalfDown,
       lookAtBall,
       lookBallGoal,
       readyAndLocate,
       lookAtTarget,
       lookForwardZeroTilt,
       lookLeftZeroTilt,
       lookRightZeroTilt,
       scanLeftRight,
       scanLeftRightKeeper,
       lookForFeatures,
       walkScan)
};

HeadControl::Mode theHeadControlMode; /**< The head control mode executed by the option HeadControl. */
float lookTargetAngle;
