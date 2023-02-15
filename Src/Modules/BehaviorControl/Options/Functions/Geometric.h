/**
 * @file Geometric.h
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <math.h>
#include <cmath>
#include <string.h>

Vector2<> gloToRel(Vector2<> target) {
  Vector2<> ret;
  float dist = float(sqrt(pow(target[0] - theRobotPose.translation.x, 2) + pow(target[1] - theRobotPose.translation.y, 2)));
  float alpha =
    theRobotPose.rotation - atan2f(target[1] - theRobotPose.translation.y, target[0] - theRobotPose.translation.x);

  ret[0] = dist * cosf(alpha);
  ret[1] = -dist * sinf(alpha);
  return ret;
}

Vector2<> relToGlo(Vector2<> target) {
  Vector2<> ret;
  float dist = target.abs();
  float alpha = atan2f(-target[1], target[0]);
  float beta = theRobotPose.rotation - alpha;

  ret[0] = theRobotPose.translation.x + dist * cosf(beta);
  ret[1] = theRobotPose.translation.y + dist * sinf(beta);

  return ret;
}

// ============ Vectors ===================
Vector2<> meToGoalRel() {
  const Vector2<> opponentGoalLocation(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);
  return (opponentGoalLocation - theRobotPose.translation);
}

Vector2<> ownBallToGoalRel() {
  const Vector2<> opponentGoalLocation(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);
  return (gloToRel(opponentGoalLocation) - theBallModel.estimate.position);
}

Vector2<> ownBallToTargetRel(Vector2<> target) {
  return (target - relToGlo(theBallModel.estimate.position));
}

Vector2<> meToWorldBallRel() {
  return gloToRel(theCombinedWorldModel.ballState.position - theRobotPose.translation);
}
// ========================================

float angleToGlobalTarget(Vector2<> globalTarget) {
  return (theRobotPose.invert() * globalTarget).angle();
}

Vector2<> futureBallPosition(char modelType, float time) {
  /**
   * type:
   * m for the own estimate of the ball position (MY)
   * w for the combined estimate (WORLD)
   * */

  Vector2<> pos;
  Vector2<> vel;
  Vector2<> acc;
  Vector2<> newPositionEstimate;
  float f = theFieldDimensions.ballFriction;

  if (modelType == 'm') {
    pos = theBallModel.estimate.position;
    vel = theBallModel.estimate.velocity;
  } else if (modelType == 'w') {
    pos = theCombinedWorldModel.ballState.position;
    vel = theCombinedWorldModel.ballState.velocity;
  } else {
    OUTPUT(idText, text, "Not a valid ball model type");
  }

  newPositionEstimate.x = pos.x + (1 / f) * vel.x * (exp(f * time) - 1.f);
  newPositionEstimate.y = pos.y + (1 / f) * vel.y * (exp(f * time) - 1.f);

  return newPositionEstimate;
}

bool isWithinTol(const Pose2D& deviation, const Pose2D& tol) {
  return std::abs(deviation.rotation) < tol.rotation && std::abs(deviation.translation.x) < tol.translation.x &&
         std::abs(deviation.translation.y) < tol.translation.y;
}

bool ballInKeeperClearArea() {
  // tolerance values away from penalty area in mm
  float tolX = 250.f;
  float tolY = 0.f;
  // coordinates of the ball to keeper
  float x = theCombinedWorldModel.ballState.position.x;
  float y = theCombinedWorldModel.ballState.position.y;
  return x >= theFieldDimensions.xPosOwnGroundline && x <= theFieldDimensions.xPosOwnGoalBox + tolX &&
         std::abs(y) <= theFieldDimensions.yPosLeftGoalBox + tolY;
}

static Vector2<> otherRelToGlo(Vector2<> target, Pose2D pose) {
  Vector2<> ret;
  float dist = target.abs();
  float alpha = atan2f(-target[1], target[0]);
  float beta = pose.rotation - alpha;

  ret[0] = pose.translation.x + dist * cosf(beta);
  ret[1] = pose.translation.y + dist * sinf(beta);

  return ret;
}
