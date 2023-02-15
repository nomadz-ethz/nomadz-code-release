/**
 * @file GetUp.h
 *
 * This option lets the robot stand up when it has fallen down.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(GetUp) {
  initial_state(lying) {
    transition {
      if (theFallDownState.direction == FallDownState::back) {
        goto lyingOnBack;
      } else { /*if (theFallDownState.direction == FallDownState::front)*/
        goto lyingOnFront;
      }
      /*else if (theFallDownState.direction == FallDownState::left
        || theFallDownState.direction == FallDownState::right)
        goto lyingOnSide;*/
    }
    action {}
  }

  state(lyingOnBack) {
    transition {
      if (theMotionInfo.motionRequest.motion == MotionRequest::specialAction &&
          theMotionInfo.motionRequest.specialActionRequest.specialAction == SpecialActionRequest::getUpBackNao22) {
        goto standing;
      }
    }
    action { SpecialAction(SpecialActionRequest::getUpBackNao22, false); }
  }

  state(lyingOnFront) {
    transition {
      if (theMotionInfo.motionRequest.motion == MotionRequest::specialAction &&
          theMotionInfo.motionRequest.specialActionRequest.specialAction == SpecialActionRequest::getUpFrontNao22) {
        goto standing;
      }
    }
    action { SpecialAction(SpecialActionRequest::getUpFrontNao22, false); }
  }

  // state(lyingOnSide)
  // {
  //   transition
  //   {
  //     if (state_time > 1000)
  //     {
  //       if (theFallDownState.direction == FallDownState::back)
  //         goto lyingOnBack;
  //       else if (theFallDownState.direction == FallDownState::front)
  //         goto lyingOnFront;
  //     }
  //     else if (state_time > 3000)
  //     {
  //       goto standing;
  //     }
  //   }
  //   action
  //   {
  //     SpecialAction(SpecialActionRequest::standUpSideNao,theFallDownState.direction == FallDownState::right);
  //   }
  // }

  target_state(standing) {
    transition {
      if (theMotionInfo.motionRequest.motion != MotionRequest::specialAction &&
          theFallDownState.state == FallDownState::onGround) {
        if (theFallDownState.direction == FallDownState::back) {
          goto lyingOnBack;
        } else { /*if (theFallDownState.direction == FallDownState::front)*/
          goto lyingOnFront;
        }
        /*else if (theFallDownState.direction == FallDownState::left
          || theFallDownState.direction == FallDownState::right)
          goto lyingOnSide;*/
      }
    }
    action { WalkAtSpeed(Pose2D()); }
  }
}
