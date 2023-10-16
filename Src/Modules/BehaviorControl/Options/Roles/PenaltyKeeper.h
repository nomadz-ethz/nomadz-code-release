/**
 * @file PenaltyKeeper.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(PenaltyKeeper) {

  const float distanceToPenaltyMark = theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyMark;
  const float sitdownBreadth = 360.f;
  float sidewardsShotDist = theBallModel.estimate.position.y - theBallModel.estimate.velocity.y /
                                                                 theBallModel.estimate.velocity.x *
                                                                 theBallModel.estimate.position.x;

  initial_state(start) {
    transition { goto detectShot; }
    action { theHeadControlMode = HeadControl::lookForward; }
  }

  state(detectShot) {
    transition {
      // distance between ball and keeper
      float distanceKB = theBallModel.estimate.position.abs();

      // ball has been moved from the penalty mark and moves toward us
      if (distanceKB <= distanceToPenaltyMark * 0.9f && theBallModel.estimate.velocity.x < 0.f && theBallModel.valid &&
          state_time > 2000.f) {
        goto saveShot;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      SpecialAction(SpecialActionRequest::sitDownKeeper);
    }
  }

  state(saveShot) {
    transition {}
    action {
      if (std::abs(sidewardsShotDist) < sitdownBreadth / 2.f) {
        SpecialAction(SpecialActionRequest::sitDownKeeper);
      } else if (sidewardsShotDist >= 0) {
        SpecialAction(SpecialActionRequest::keeperJumpLeft, false); // jump left
      } else {
        SpecialAction(SpecialActionRequest::keeperJumpLeft, true); // jump right
      }
    }
  }
}
