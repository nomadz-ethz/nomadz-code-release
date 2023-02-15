/**
 * @file PenaltyStriker.h
 *
 * Penalty striker role
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(PenaltyStriker) {

  initial_state(searchForBall) {
    transition {
      if (state_time > 1000.f) {
        bool pickedLeftCorner = PickCornerForPenalty();
        if (pickedLeftCorner) {
          PlaySound("left.wav");
          goto penaltyKickLeft;
        } else {
          PlaySound("right.wav");
          goto penaltyKickRight;
        }
      }
    }
    action {
      theHeadControlMode = HeadControl::walkScan;
      Stand();
    }
  }

  state(penaltyKickLeft) {
    transition {
      if (action_done) {
        goto searchForBall;
      }
    }
    action {
      const float xHackOffset = 25.f;
      Vector2<> globalTargetPenaltyStriker =
        Vector2<>(sgn(theRobotPose.translation.x) * theFieldDimensions.xPosOpponentGroundline,
                  sgn(theRobotPose.translation.x) * (theFieldDimensions.yPosLeftGoal - xHackOffset - 150.f));
      theHeadControlMode = HeadControl::lookAtBall;
      KickTo(globalTargetPenaltyStriker);
    }
  }

  state(penaltyKickRight) {
    transition {
      if (action_done) {
        goto searchForBall;
      }
    }
    action {
      const float yHackOffset = 125.f; // m
      Vector2<> globalTargetPenaltyStriker =
        Vector2<>(sgn(theRobotPose.translation.x) * theFieldDimensions.xPosOpponentGroundline,
                  sgn(theRobotPose.translation.x) * (theFieldDimensions.yPosRightGoal + 150.f + yHackOffset));
      theHeadControlMode = HeadControl::lookAtBall;
      KickTo(globalTargetPenaltyStriker);
    }
  }
}
