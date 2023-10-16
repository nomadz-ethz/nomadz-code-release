/**
 * @file SupportPlay.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(SupportPlay) {
  const Vector2<> gloOpponentGoal(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);
  Vector2<> gloBall = libWorldModel.ballPosition();

  initial_state(waitForBall) {
    transition {}
    action { TravelTo(theFieldPosition.currentTargetPose); }
  }
}
