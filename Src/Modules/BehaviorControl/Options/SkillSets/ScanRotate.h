/**
 * @file SearchForBall.h
 *
 * Complete optimizable algorithm for finding the ball. Used in Nagoya 2017
 * If the search was successful the behaviour will tranistion into target_state.
 * If the search was not sucessful the behaviour will transition aborted_state.
 * However the body will not turn towards the ball. This is the task of the
 * behaviour which implements SearchForBall.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(ScanRotate) {
  const bool verbose = false;

  static int rotateCount;
  static int numberOfRotationPoints;
  static int turningAngleDegrees;

  initial_state(start) {
    transition {
      if (state_time > 100) {
        if (verbose) {
          OUTPUT(idText, text, "initial: rotateCount = " << rotateCount);
        }
        goto rotate;
      }
    }
    action {
      rotateCount = 0;
      numberOfRotationPoints = 4;
      turningAngleDegrees = 360 / numberOfRotationPoints;
      Stand();
    }
  }

  /*State to look for the ball by turning the head*/
  state(scan) {
    transition {
      if (rotateCount < numberOfRotationPoints) {
        if (state_time > 2000) {
          goto rotate;
        }
      } else {
        goto failure;
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      Stand();
    }
  }

  /*State to rotate the body*/
  state(rotate) {
    transition {
      /*TODO find smarter transition condition.
       * Action done will not work because WalkToTarget enters the target_state
       * as soon as the motion request is sent. It does NOT wait until the
       * motion has completed.*/
      if (state_time > 2000) {
        ++rotateCount;
        if (verbose) {
          OUTPUT(idText, text, "rotate: rotateCount = " << rotateCount << "; goto scan");
        }
        goto scan;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2D(0.5f, 0.f, 0.f), Pose2D(fromDegrees(turningAngleDegrees), 0.f, 0.f));
    }
  }

  /** ball was seen this state can be observed by the higher level state machine using action_done*/
  target_state(success) {}

  /* ball could not be found this state can be observed by the higher level state machine using action_aborted*/
  aborted_state(failure) {}
}

// TODO Remove superfluous function
// Vector2<> generateTargetNearCenter() {
//    const bool side = (rand() % 2 == 1);
//    const float centerRadius = theFieldDimensions.centerCircleRadius;
//    return gloToRel(Vector2<>(0.f, side ? (-centerRadius - rand() % 1000) : (centerRadius + rand() % 1000)));
//}
