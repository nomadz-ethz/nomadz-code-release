/**
 * @file LookAndLocate.h
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

float worldBallAngle;
int previous;

option(LookAndLocate) {
  initial_state(lookOpponentGoal) {
    transition {
      if (state_time > 2000) {
        previous = 1;
        goto lookBall;
      }
    }
    action {
      SetHeadPanTilt(libCodeRelease.angleToGoal, 0.38f, fromDegrees(150.f)); // can be adjusted
    }
  }
  state(lookBall) { // TODO(luliu):Use this in guidedSearch->WalkScan?
    transition {
      if (state_time > 5000)
        goto next;
    }
    action {

      if (gloToRel(theCombinedWorldModel.ballState.position).x > 0) {
        worldBallAngle = meToWorldBallRel().angle();
      } else {
        worldBallAngle = meToWorldBallRel().angle() + fromDegrees(180.f);
      }

      SetHeadPanTilt(worldBallAngle, 0.38f, fromDegrees(150.f));
    }
  }
  state(lookOwnGoal) {
    transition {
      if (state_time > 2000) {
        previous = 2;
        goto lookBall;
      }
    }
    action {
      SetHeadPanTilt(libCodeRelease.angleToOwnGoal, 0.38f, fromDegrees(150.f)); // can be adjusted
    }
  }
  state(next) {
    transition {
      switch (previous) {
      case 1:
        goto lookOwnGoal;
      case 2:
        goto lookOpponentGoal;
      }
    }
  }
}
