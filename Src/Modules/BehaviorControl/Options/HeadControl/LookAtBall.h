/**
 * @file LookAtBall.h
 *
 * Stable dynamic head motion. Looks at ball all the time, as long as it was seen in the last 3s.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(LookAtBall) {
  initial_state(lookAtBall) {
    action {
      Vector2<> ballPosition =
        (theFrameInfo.getTimeSince(theBallModel.timeWhenLastDetected) < 200.f ? theBallModel.lastPerception
                                                                              : theBallModel.estimate.position);
      Vector2<> combinedBallPosition = gloToRel(theCombinedWorldModel.ballStateOthers.position);

      // I see ball
      if (libCodeRelease.timeSinceBallWasSeen() < 2500.f) {

        // Stare at ball
        if (ballPosition.abs() > 300.f) {
          const float pan = ballPosition.angle();
          const float maxSpeed = (std::abs(pan) > fromDegrees(5.f)) ? 120.f : 0.f;
          SetHeadTargetOnGround(
            {ballPosition.x, ballPosition.y, 0.f}, HeadMotionRequest::autoCamera, true, fromDegrees(maxSpeed));

          // Ball is really close; limit range (and maybe speed) of panning & look down
        } else {
          const float pan = std::max(-0.33f, std::min(ballPosition.angle(), 0.33f));
          const float maxSpeed = (std::abs(pan) > fromDegrees(15.f)) ? 90.f : 0.f;
          SetHeadPanTilt(pan, 0.40f, fromDegrees(maxSpeed));
        }

        // Lost ball; look at combined location
      } else if (theCombinedWorldModel.timeSinceBallLastSeenOthers < 5000.f) {
        const float pan = combinedBallPosition.angle();
        const float maxSpeed = (std::abs(pan) > fromDegrees(5.f)) ? 120.f : 0.f;

        SetHeadTargetOnGround(
          {combinedBallPosition.x, combinedBallPosition.y, 0.f}, HeadMotionRequest::autoCamera, true, fromDegrees(maxSpeed));

        // Nobody sees ball; look for it
      } else {
        // Start looking in the direction of the last place we see it.
        if (ballPosition.y > 0) {
          ScanLeftRight();
        } else {
          ScanRightLeft();
        }
      }
    }
  }
}
