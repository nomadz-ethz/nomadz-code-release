/**
 * @file DefenderPosition.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(DefenderPosition) {
  Vector2<> gloBall = libWorldModel.ballPosition();
  // calculation of optimal positions for creating barrier
  const float lBarrier = 600.0f;
  const float lGoal = (float)theFieldDimensions.yPosLeftGoal * 2;
  float xBall = std::abs(theFieldDimensions.xPosOwnGoal - gloBall.x);
  float xW = xBall * (1.0f - lBarrier / lGoal) + theFieldDimensions.xPosOwnGoal;
  // xBarrier > xPosOwnGoalBox, to avoid defenders entering into goal box
  xW = (xW <= (theFieldDimensions.xPosOwnGoalBox + 250.f)) ? (theFieldDimensions.xPosOwnGoalBox + 250.f) : xW;
  xW = (xW >= (theFieldDimensions.xPosHalfWayLine - 2000.f)) ? (theFieldDimensions.xPosHalfWayLine - 2000.f) : xW;
  float tanBall = (theFieldDimensions.yPosCenterGoal - gloBall.y) / (theFieldDimensions.xPosOwnGoal - gloBall.x);
  float angleBall = atan2(gloBall.y - theFieldDimensions.yPosCenterGoal, gloBall.x - theFieldDimensions.xPosOwnGoal);
  float yW = tanBall * (xW - gloBall.x) + gloBall.y; // stand on the line between the ball and the center of goal
  float relAngleBall = angleToGlobalTarget(libWorldModel.ballPosition());

  Vector2<> posToPlace =
    (theRobotInfo.number % 2 == 0) ? Vector2<>(xW, yW + lBarrier / 2.) : Vector2<>(xW, yW - lBarrier / 2.);
  Vector2<> relPosToPlace = gloToRel(posToPlace);

  initial_state(alert) {

    transition {
      if ((std::fabs(relPosToPlace.x) > 50.f || std::fabs(relPosToPlace.y) > 50.f ||
           std::fabs(relAngleBall) > fromDegrees(10.f))) {
        goto posToDefend;
      }
    }

    action {
      theHeadControlMode = HeadControl::lookAtBall;
      Stand();
    }
  }

  state(posToDefend) {

    transition {
      if (std::fabs(relPosToPlace.x) < 50.f && std::fabs(relPosToPlace.y) < 50.f &&
          std::fabs(relAngleBall) < fromDegrees(10.f)) {
        goto alert;
      }
    }

    action {
      theHeadControlMode = HeadControl::lookAtBall;
      float targetDist = (posToPlace - theRobotPose.translation).abs();
      if (targetDist < 1500.f && std::fabs(relAngleBall) < fromDegrees(25.f) &&
          std::fabs(theRobotPose.rotation) < fromDegrees(60.f) && gloBall.x > theRobotPose.translation.x) {
        WalkToTarget(Pose2D{0.6f, 0.6f, 0.4f}, Pose2D{0, relPosToPlace});
      } else {
        TravelTo(Pose2D(angleBall, posToPlace));
      }
    }
  }
}
