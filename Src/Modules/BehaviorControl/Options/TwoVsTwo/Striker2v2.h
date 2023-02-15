/**
 * @file Striker2v2.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iostream>
#include <vector>

option(Striker2v2) {

  /*
  common_transition {
    if () {
      goto ;
    }
  }*/

  initial_state(start) {
    transition {
      if (state_time > 500) {
        goto play;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }

  state(play) {
    transition {
      // If the ball was not seen after a certain time and no search is in progress search for ball
      if (action_aborted) {
        goto search;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      OffensivePlay2v2();
    }
  }

  state(search) {
    transition {
      if (action_done) {
        goto play;
      }
      if (action_aborted) {
        goto relocate;
      }
    }
    action { StrikerSearch(); }
  }

  state(relocate) {
    transition {
      if (action_done) {
        goto search;
      }
    }
    action { Relocate(); }
  }
}