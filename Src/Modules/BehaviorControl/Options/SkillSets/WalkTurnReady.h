/**
 * @file WalkTurnReady.h
 *
 * The robot will walk towards the assigned position and then rotate so that
 * it's facing the ball, which we for now consider optimal.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(WalkTurnReady, Vector2<> globalTarget) {
  const float bound = 50.f;
  const float angleBound = fromDegrees(10.f);
  initial_state(initial) {
    transition {
      if (state_time > 10) {
        goto walkToPosition;
      }
    }
    action {}
  }

  state(walkToPosition) {
    transition {
      if ((std::abs(globalTarget.x - theRobotPose.translation.x) < bound) &&
          (std::abs(globalTarget.y - theRobotPose.translation.y) < bound) &&
          (std::abs(theRobotPose.rotation) < angleBound)) {
        goto ready;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForFeatures;

      // Walk straight to the target, as quickly as possible
      WalkTo(Pose2D(0.f, globalTarget));
    }
  }

  state(ready) { // When the robot is at its destination pose, keep looking around to gather as
                 // much information as possible.
    transition { // If it realizes that it's in the wrong position (quite likely!), it should reposition itself.
      if ((std::abs(globalTarget.x - theRobotPose.translation.x) > 3 * bound) ||
          (std::abs(globalTarget.y - theRobotPose.translation.y) > 3 * bound)) {
        goto initial;
      }
    }
    action {
      Stand();
      theHeadControlMode = HeadControl::lookForFeatures;
    }
  }
}
