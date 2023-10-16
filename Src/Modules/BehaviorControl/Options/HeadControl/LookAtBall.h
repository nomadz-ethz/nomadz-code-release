/**
 * @file LookAtBall.h
 *
 * Stable dynamic head motion. Looks at ball all the time, as long as it was seen in the last 3s.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(LookAtBall) {
  initial_state(lookAtBall) {
    action {
      Vector2<> ballPosition = theBallModel.valid ? theBallModel.lastPerception : theBallModel.estimate.position;
      Vector2<> combinedBallPosition = gloToRel(theCombinedWorldModel.ballStateOthers.position);

      // I see ball
      if (!theBallModel.lost) {
        // Stare at ball
        SetHeadTargetOnGround({ballPosition.x, ballPosition.y, 0.f}, HeadMotionRequest::autoCamera, true, defaultSpeed);

        // Lost ball; look at combined location
      } else if (theCombinedWorldModel.ballIsValidOthers) {
        SetHeadTargetOnGround(
          {combinedBallPosition.x, combinedBallPosition.y, 0.f}, HeadMotionRequest::autoCamera, true, defaultSpeed);

        // Nobody sees ball; look for it
      } else {
        WalkScan();
      }
    }
  }
}
