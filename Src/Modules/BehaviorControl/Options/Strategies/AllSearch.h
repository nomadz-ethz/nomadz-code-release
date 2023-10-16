/**
 * @file AllSearch.h
 *
 * Search the ball with cooperative field coverage and RL trained targets on the blockchain.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include <iostream>
#include <vector>

option(AllSearch) {

  Vector2<> gloBall;
  Vector2<> relBall;
  static Vector2<> oldBall;
  static float randDisp;
  const float neighborhood = (theBehaviorStatus.role == BehaviorStatus::keeper) ? 100. : 500.;
  const std::vector<float> posAdjustment = {0., 1000., -1000.};
  static uint posCounter = 0;

  common_transition {
    if (theBallModel.valid) {
      goto success;
    }
    if (theRobotPose.lost) {
      goto lost;
    }
  }

  initial_state(noGlobalBall) {
    transition {
      if (theCombinedWorldModel.ballIsValidOthers) {
        goto globalBall;
      }
      if (theBehaviorStatus.role != BehaviorStatus::keeper) {
        if ((abs(theFieldPosition.searchPose.translation.x - theRobotPose.translation.x) +
             abs(theFieldPosition.searchPose.translation.y + posAdjustment[posCounter] - theRobotPose.translation.y)) <
            neighborhood) {
          goto rotate;
        }
      }
      if (theBehaviorStatus.role == BehaviorStatus::keeper) {
        if ((abs(theFieldPosition.searchPose.translation.x - theRobotPose.translation.x) +
             abs(theFieldPosition.searchPose.translation.y - theRobotPose.translation.y)) < neighborhood) {
          goto rotate;
        }
      }
    }
    action {

      if (theBehaviorStatus.role != BehaviorStatus::keeper) {
        TravelTo(Pose2D(theFieldPosition.searchPose.rotation,
                        theFieldPosition.searchPose.translation.x,
                        theFieldPosition.searchPose.translation.y + posAdjustment[posCounter]));
      } else {
        TravelTo(theFieldPosition.searchPose);
      }

      theHeadControlMode = HeadControl::lookForward;
    }
  }

  state(rotate) {
    transition {
      if (action_aborted) {
        if (theBehaviorStatus.role != BehaviorStatus::keeper) {
          posCounter = (posCounter < 2) ? posCounter + 1 : 0;
        }
        goto noGlobalBall;
      }
      if (theCombinedWorldModel.ballIsValidOthers) {
        goto globalBall;
      }
    }
    action {
      // FIXME: should we move somewhere between consecutive scans?
      ScanTheField();
    }
  }

  /////////////////////// NOT USED NOW /////////////////////////////////////////////////////////

  state(globalBall) {
    transition {
      if (!theCombinedWorldModel.ballIsValidOthers) {
        goto noGlobalBall;
      }

      if (state_time > 3000) {
        oldBall = libWorldModel.ballPosition();
        randDisp = 2 * (std::rand() - RAND_MAX / 2) /
                   static_cast<float>(RAND_MAX); // in [-1,1], seems to be the same for every robot in simulation
        goto enlargeSearch;
      }
    }
    action {
      gloBall = libWorldModel.ballPosition();
      relBall = gloToRel(gloBall);
      WalkToTarget(Pose2D(1.f, 0.f, 0.f), Pose2D(relBall.angle(), 0.f, 0.f));

      theHeadControlMode = HeadControl::lookAtBall;
    }
  }

  // Need a logic to address the possible mismatch between global and local location

  state(enlargeSearch) {
    transition {
      Vector2<> newBall = libWorldModel.ballPosition();
      if ((abs(oldBall.x - newBall.x) + abs(oldBall.y - newBall.y)) > 1500) {
        goto globalBall;
      }
      if (!theCombinedWorldModel.ballIsValidOthers && theCombinedWorldModel.timeSinceBallLastSeenOthers > 5100) {
        goto noGlobalBall;
      }
    }
    action {
      gloBall = libWorldModel.ballPosition();

      Pose2D target = Pose2D(0., gloBall.x + 1000 * randDisp, gloBall.y + 1000 * randDisp);
      TravelTo(target);
      theHeadControlMode = HeadControl::walkScan;
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  target_state(success) {}

  aborted_state(lost) {}
}
