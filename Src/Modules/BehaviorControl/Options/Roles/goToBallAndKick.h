/**
 * @file goToBallAndKick
 *
 * A test role to simply go to the ball and kick it
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(goToBallAndKick) {
  Vector2<> target;

  initial_state(start) {
    transition {
      if (state_time > 100) {
        goto search;
      }
    }
    action {}
  }

  state(attack) {
    transition {
      if (action_done || action_aborted) {
        goto search;
      }
    }
    action {
      target = Vector2<>(0, 0); // uncomment this to shoot the ball towards the field center
      WalkAndKick(target);
    }
  }

  state(search) {
    transition {
      if (libCodeRelease.timeSinceBallWasSeen() < 300) {
        target = relToGlo(theBallModel.estimate.position + Vector2<>(theBallModel.estimate.position).normalize(100.f));
        goto attack;
      }
    }
    action {
      theHeadControlMode = HeadControl::scanLeftRight;
      ScanRotate();
    }
  }
}
