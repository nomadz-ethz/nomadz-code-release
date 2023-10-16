/**
 * @file ScanTheField.h
 *
 * Complete optimizable algorithm for finding the ball. Used in Bordeaux 2023 with bad results.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

option(ScanTheField) {

  static int rotateCount;
  static int numberOfRotationPoints;
  static int turningAngleDegrees;

  const std::vector<float> keeperAngles = {0., 60., 0., -60.};

  initial_state(start) {
    transition {
      if (state_time > 100) {
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
      theHeadControlMode = HeadControl::scanLeftRight;
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
        goto scan;
      }
    }
    action {
      theHeadControlMode = HeadControl::lookForward;

      if (theBehaviorStatus.role == BehaviorStatus::keeper) {
        WalkToTarget(Pose2D(1.f, 0.f, 0.f),
                     Pose2D(fromDegrees(keeperAngles[rotateCount]) - theRobotPose.rotation, 0.f, 0.f));
      } else {
        WalkToTarget(Pose2D(1.f, 0.f, 0.f), Pose2D(fromDegrees(turningAngleDegrees), 0.f, 0.f));
      }
    }
  }

  /* ball could not be found this state can be observed by the higher level state machine using action_aborted*/
  aborted_state(failure) {}
}
