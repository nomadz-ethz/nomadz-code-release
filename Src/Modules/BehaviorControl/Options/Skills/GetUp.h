/**
 * @file GetUp.h
 *
 * This option lets the robot stand up when it has fallen down.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */
int numOfGetUpAttempts;

option(GetUp) {

  initial_state(lying) {
    transition {
      if (state_time > 500) {
        if (numOfGetUpAttempts >= 3) {
          goto fallenRobot;
        }
        numOfGetUpAttempts++;
        // OUTPUT_TEXT("number of get up attempts " << numOfGetUpAttempts);
        if (theFallDownState.direction == FallDownState::back) {
          goto lyingOnBack;
        } else { /*if (theFallDownState.direction == FallDownState::front)*/
          goto lyingOnFront;
        }
      }
    }
    // Take a break. Good for the joints
    action { SpecialAction(SpecialActionRequest::relax, false); }
  }

  state(lyingOnBack) {
    transition {
      if (theMotionInfo.motionRequest.motion == MotionRequest::specialAction &&
          theMotionInfo.motionRequest.specialActionRequest.specialAction == SpecialActionRequest::getUpBackNao22) {
        goto gettingUP;
      }
    }
    action { SpecialAction(SpecialActionRequest::getUpBackNao22, false); }
  }

  state(lyingOnFront) {
    transition {
      if (theMotionInfo.motionRequest.motion == MotionRequest::specialAction &&
          theMotionInfo.motionRequest.specialActionRequest.specialAction == SpecialActionRequest::getUpFrontNao22) {
        goto gettingUP;
      }
    }
    action { SpecialAction(SpecialActionRequest::getUpFrontNao22, false); }
  }

  state(gettingUP) {
    transition {
      if (theFallDownState.state == FallDownState::upright) {
        goto checkUprightStability;
      } else if (state_time > 3500) { // theSpecialActionsOutput.isMotionDone seems to be done anything but its name
                                      // sugguest. state_time as a quick fix
        goto lying;
      }
    }
    action {}
  }

  state(checkUprightStability) {
    transition {
      if (theFallDownState.state == FallDownState::upright) {
        if (state_time > 1000) {
          goto standing;
        }
      } else if (theFallDownState.state == FallDownState::onGround)
        if (state_time > 100) {
          goto lying;
        }
    }
    action {}
  }

  aborted_state(fallenRobot) {
    transition {
      if (theFallDownState.state == FallDownState::upright) {
        goto checkUprightStability;
      }
    }
    action { SpecialAction(SpecialActionRequest::relax, false); }
  }

  target_state(standing) {
    transition {}
    action {
      numOfGetUpAttempts = 0;
      WalkAtSpeed(Pose2D());
    }
  }
}
