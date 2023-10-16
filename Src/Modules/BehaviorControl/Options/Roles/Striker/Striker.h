/**
 * @file Striker.h
 *
 * Striker state machine
 *
 *               ----------
// *             | search |  <---       theBallModel.lost               <-|
 *               ----------                                               |
 *                     |                                                  |
 *                     |  theBallModel.valid                              |
 *                    \|/                                                 |
 *    -------       ------                                                |
 *   | Start | <-> | play |-----------------------------------------------|
 *    -------       ------
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include <iostream>
#include <vector>

option(Striker) {
  initial_state(start) {
    transition {
      if (state_time > 250) {
        goto play;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }

  state(play) {
    transition {}
    action {
      theHeadControlMode = HeadControl::lookAtBall;
      OffensivePlay();
    }
  }
}
