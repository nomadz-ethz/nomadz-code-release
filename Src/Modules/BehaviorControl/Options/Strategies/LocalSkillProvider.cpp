/**
 * @file LocalSkillProvider.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include "LocalSkillProvider.h"

MAKE_MODULE(LocalSkillProvider, Behavior Control);

LocalSkillProvider::LocalSkillProvider() {}

void LocalSkillProvider::update(LocalSkill& localSkill) {
  DECLARE_DEBUG_DRAWING3D("module:LocalSkill:bestFreeRange", "field");
  localSkill.skillType = LocalSkill::Skill::none;
  localSkill.nextTargetBallAngle = float();
  localSkill.targetProperty = LocalSkill::TargetProperty::freeRange;
  localSkill.bestFreeRangesMin = float();
  localSkill.bestFreeRangesMax = float();
  // Only compute the skill when you have the ball
  if (thePersonalData.hasBallLock) {
    skillType = decideSkillType();
    nextTargetBallAngle = calcNextTargetBallAngle(getFreeRanges());
    localSkill.targetProperty = decideTargetProperty(isCloseToGoal(), skillType);
    localSkill.skillType = skillType;
    localSkill.nextTargetBallAngle = nextTargetBallAngle;
    localSkill.bestFreeRangesMin = toDegrees(bestFreeRangesMin);
    localSkill.bestFreeRangesMax = toDegrees(bestFreeRangesMax);
    draw();
  }
}

Vector2<> LocalSkillProvider::calcReferenceTargetBallPosition() {
  return Vector2<>(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);
}

LocalSkill::Skill LocalSkillProvider::decideSkillType() {
  if (isCloseToGoal()) {
    return LocalSkill::fastKick;
  } else if (isInDanger()) {
    return LocalSkill::inWalkKick;
  } else {
    return LocalSkill::fastKick;
  }
}

LocalSkill::TargetProperty LocalSkillProvider::decideTargetProperty(const bool isCloseToGoal,
                                                                    LocalSkill::Skill skillType) const {
  if (isCloseToGoal == false && skillType == LocalSkill::inWalkKick) {
    // is closeToGoal regardless of in danger or not always shoots on goal
    return LocalSkill::freeRange;
  } else {
    return LocalSkill::goalCenter;
  }
}

std::list<Range<float>> LocalSkillProvider::getFreeRanges() {
  const Vector2<> gloBall = theRobotPose * theBallModel.estimate.position;
  const Vector2<> goalLeft = {theFieldDimensions.xPosOpponentGroundline,
                              theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius};
  const Vector2<> goalRight = {theFieldDimensions.xPosOpponentGroundline,
                               theFieldDimensions.yPosRightGoal + theFieldDimensions.goalPostRadius};
  const Range<float> goalRange = {(goalRight - gloBall).angle(), (goalLeft - gloBall).angle()};
  // global frame
  return calcFreeRanges(gloBall, goalRange);
}

float LocalSkillProvider::calcNextTargetBallAngle(std::list<Range<float>> freeRanges) {
  const Vector2<> gloBall = theRobotPose * theBallModel.estimate.position;
  const float robotToBallAngle = (gloBall - theRobotPose.translation).angle();
  if (freeRanges.empty()) {
    return (theRobotPose.invert() * calcReferenceTargetBallPosition()).angle();
  } else {
    Range<float> bestFreeRange = getBestFreeRange(freeRanges, robotToBallAngle);
    bestFreeRangesMin = bestFreeRange.min;
    bestFreeRangesMax = bestFreeRange.max;
    float targetRange = bestFreeRange.getCenter();
    // angle w.r.t to the center of the best free goal range in robot frame
    return targetRange - theRobotPose.rotation;
  }
}

bool LocalSkillProvider::isCloseToGoal() const {
  return std::pow(theFieldDimensions.xPosOpponentGroundline - theRobotPose.translation.x, 2) +
           std::pow(theRobotPose.translation.y, 2) <
         std::pow(theFieldDimensions.xPosOpponentGroundline,
                  2); // circle with radius spanning from center of goal to origin
}

bool LocalSkillProvider::isInDanger() const {
  return nearestOpponentInSight(0.8, 600.f) < 800.f;
}

float LocalSkillProvider::nearestOpponentInSight(float angle, float offset) const {
  if (thePlayerModel.players.empty()) {
    return INFINITY; // no opponent in sight
  }

  std::vector<Vector2<>> players;
  for (const auto& player : thePlayerModel.players) {
    players.push_back(theRobotPose * player.relPosOnField);
  }

  std::vector<bool> playersAreTeamMates = isTeamMate(players);

  float minDistance = INFINITY;
  for (int i = 0; i < static_cast<int>(thePlayerModel.players.size()); i++) {
    const auto& localPos = thePlayerModel.players[i].relPosOnField;
    if (localPos.x <= 20 || playersAreTeamMates[i]) {
      continue;
    }

    if (std::abs(localPos.angle()) < angle && std::abs(localPos.y) < offset) {
      minDistance = std::min(minDistance, localPos.abs());
    }
  }

  return minDistance;
}

std::vector<bool> LocalSkillProvider::isTeamMate(const std::vector<Vector2<>>& players) const {
  // input argument players contains robot positions in global frame
  std::vector<int> teamMateIDs = getTeamMateIDs();

  // is TeamMate based on TeamMateData
  std::vector<bool> isTeamMate(players.size(), false);
  const float tol = 200.f; // TODO: find a reasonable tolerance
  for (const auto& id : teamMateIDs) {
    const Vector2<> teamMatePos = theTeamMateData.robotPoses[id].translation;
    float minDeviation = INFINITY;
    int minIndex = 0;

    for (int i = 0; i < static_cast<int>(players.size()); i++) {
      float deviation = (teamMatePos - players[i]).abs(); // is teamMatePos in global frame?
      if (deviation < minDeviation) {
        minDeviation = deviation;
        minIndex = i;
      }
    }
    if (minDeviation < tol) {
      isTeamMate[minIndex] = true;
    }
  }

  return isTeamMate;
}

std::list<Range<float>> LocalSkillProvider::calcFreeRanges(const Vector2<>& ball, const Range<float>& goalRange) {
  // Represents a union of intervals of angles (defined in global frame centered on the ball)
  // that represent free regions when aiming at the goal
  const int playerAvoidanceRadius = 240;
  std::list<Range<float>> freeRanges = {goalRange};
  if (thePlayerModel.players.size() == 0) {
    return freeRanges;
  }

  std::vector<Vector2<>> players;
  for (const auto& player : thePlayerModel.players) {
    players.push_back(theRobotPose *
                      player.relPosOnField); // transformation of relative position of each player to global frame
  }

  std::vector<bool> isTeamMate = LocalSkillProvider::isTeamMate(players);

  // Subtract a free interval for every player
  for (int i = 0; i < static_cast<int>(players.size()); i++) {
    if (isTeamMate[i]) {
      continue;
    }

    const auto& player = players[i];
    const Vector2<> ballToPlayer = player - ball;
    const float blockedAngle = std::asin(playerAvoidanceRadius / ballToPlayer.abs());

    const Range<float> blockedRange = {ballToPlayer.angle() - blockedAngle, ballToPlayer.angle() + blockedAngle};

    // Ignore players completely outside of our problem
    if (blockedRange < goalRange || blockedRange > goalRange) {
      continue;
    }

    // Remove blockedRange from [freeRanges]
    for (auto i = freeRanges.begin(); i != freeRanges.end();) {
      auto& range = *i;
      if (blockedRange.contains(range)) {
        // Delete original range
        i = freeRanges.erase(i);

      } else if (range.contains(blockedRange)) {
        // Split original range into two
        const Range<float> range1(range.min, blockedRange.min);
        const Range<float> range2(blockedRange.max, range.max);

        i = freeRanges.erase(i);
        i = freeRanges.insert(i, range1);
        ++i;
        i = freeRanges.insert(i, range2);
        ++i;

      } else {
        if (range.overlaps(blockedRange)) {
          // Cut back "max"-side of original range
          range.max = blockedRange.min;
        } else if (range.overlappedBy(blockedRange)) {
          // Cut back "min"-side of original range
          range.min = blockedRange.max;
        }
        ++i;
      }
    }
  }

  return freeRanges;
};

Range<float> LocalSkillProvider::getBestFreeRange(const std::list<Range<float>>& freeRanges, const float robotToBallAngle) {
  return *std::max_element(
    freeRanges.begin(), freeRanges.end(), [robotToBallAngle](const Range<float>& a, const Range<float>& b) {
      // Reward wide ranges, lightly penalize those far away
      const float rewardA = a.getSize() - 0.15f * std::abs(angleDifference(robotToBallAngle, a.getCenter()));
      const float rewardB = b.getSize() - 0.15f * std::abs(angleDifference(robotToBallAngle, b.getCenter()));
      return rewardA < rewardB;
    });
}

void LocalSkillProvider::draw() {
  Pose2D ballFrameMin = Pose2D(bestFreeRangesMin, theRobotPose * theBallModel.estimate.position);
  Pose2D ballFrameMax = Pose2D(bestFreeRangesMax, theRobotPose * theBallModel.estimate.position);
  Vector3<> leftLineDirectionPoint = Vector3<>(ballFrameMax * Vector2<>(2000.f, 0.f));
  Vector3<> rightLineDirectionPoint = Vector3<>(ballFrameMin * Vector2<>(2000.f, 0.f));
  Vector3<> gloBallPos = Vector3<>(theRobotPose * theBallModel.estimate.position);
  ColorRGBA color;
  if (theOwnTeamInfo.teamNumber == 2) {
    color = ColorClasses::red;
  } else {
    color = ColorClasses::blue;
  }
  LINE3D_VEC("module:LocalSkill:bestFreeRange", gloBallPos, leftLineDirectionPoint, 4, color);
  LINE3D_VEC("module:LocalSkill:bestFreeRange", gloBallPos, rightLineDirectionPoint, 4, color);
}

std::vector<int> LocalSkillProvider::getTeamMateIDs() const {
  std::vector<int> teamMateIDs = {1, 2, 3, 4, 5, 6, 7};
  teamMateIDs.erase(teamMateIDs.begin() + theRobotInfo.number - 1);
  return teamMateIDs;
}