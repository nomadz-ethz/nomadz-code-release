/**
 * @file LookBallGoal.h
 *
 * Stable dynamic head motion. Will switch between looking at ball and the opponents goal,
 * unless the head motion is awkward. Improves self and ball localization. For common usage.
 *
 * Comfortable: Headangle < 65 degrees.
 * If the angle is greater that 65, the robot will tilt his head back drastically.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#define worldBallRelativeX gloToRel(theCombinedWorldModel.ballState.position).x
#define worldBallRelativeAngle std::abs(gloToRel(theCombinedWorldModel.ballState.position).angle())
#define ownBallRelativeAngle std::abs(theBallModel.estimate.position.angle())
#define ownBallRelativeX theBallModel.estimate.position.x
#define goalSeen theFrameInfo.getTimeSince(theGoalPercept.timeWhenCompleteGoalLastSeen)
#define opponentGoalRelative                                                                                                \
  gloToRel(Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal))
#define ownGoalRelative gloToRel(Vector2<>(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosCenterGoal))

float lookAtBallAngle;

option(LookBallGoal) {
  initial_state(straight) {
    transition {
      if (state_time > 100) {
        goto lookBall;
      }
    }
    action { SetHeadPanTilt(0.f, 0.32f, fromDegrees(150.f)); }
  }

  state(lookGoal) {
    transition {
      if ((libCodeRelease.timeSinceBallWasSeen() > 4000) || (state_time > 6000) ||
          (theHeadMotionRequest.pan > fromDegrees(50.f))) {
        goto lookBall;
      }
    }
    action { /* Keep track of where the goal is and thus your location and thus indirectly a
            better ball estimate in the CombinedWorldModel.*/
      if (std::abs(opponentGoalRelative.angle()) < std::abs(ownGoalRelative.angle())) {
        SetHeadPanTilt(opponentGoalRelative.angle(), 0.32f, fromDegrees(150.f)); // look at opponent's goal
      } else {
        SetHeadPanTilt(ownGoalRelative.angle(), 0.32f, fromDegrees(150.f)); // look at own goal
      }
    }
  }

  state(lookBall) {
    transition {
      if ((goalSeen > 4000) &&
          ((opponentGoalRelative.angle() < fromDegrees(45.f)) || (ownGoalRelative.angle() < fromDegrees(45.f))) &&
          (theBallModel.estimate.position.abs() > 1000.f)) { // We are not going to crash into the ball anytime soon.
        goto lookGoal;                                       // goal not at an awkward angle and ball not too close
      }
    }
    action {
      if (std::abs(ownBallRelativeAngle) < fromDegrees(70.f)) {
        lookAtBallAngle = 5.f / 7.f * ownBallRelativeAngle; // if you see the ball, look at it
      } else if (std::abs(worldBallRelativeAngle) < fromDegrees(70.f)) {
        lookAtBallAngle = 5.f / 7.f * worldBallRelativeAngle; // if you don't see it, look at where you're told it is
      } else {
        lookAtBallAngle = 0; // if you think the ball is behind you or at an awkward angle, don't try to look
      }

      SetHeadPanTilt(lookAtBallAngle, 0.32f, fromDegrees(150.f));
    }
  }
}
