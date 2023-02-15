/**
 * @file PassTo.h
 *
 * Walk to the ball and pass it towards some position on the field, as quickly as possible.
 * On the way to the ball, walks around other players to avoid collision.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(PassTo, Vector2<> target, Pose2D tol = {fromDegrees(7.5f), 25.f, 15.f}) {

  initial_state(walk) {
    transition {
      if (action_done) {
        goto pass;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      GetBehindBall(target, kickOffsetPass, tol);
    }
  }

  state(pass) {
    transition {
      if (state_time > 3000) {
        goto done;
      }
    }
    action { DistancePass((target - relToGlo(theBallModel.estimate.position)).abs()); }
  }

  target_state(done) {
    transition {
      if (state_time > 100) {
        goto walk;
      }
    }
    action { Stand(); }
  }
}
