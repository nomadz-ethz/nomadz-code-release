/**
 * @file LibCodeRelease.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 */

#include "../LibraryBase.h"

namespace behavior {
#include "LibCodeRelease.h"

  LibCodeRelease::LibCodeRelease() : angleToGoal(0.f) {}

  void LibCodeRelease::preProcess() {
    // precompute
    angleToGoal = (theRobotPose.invert() * Vector2<>(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
    angleToOwnGoal = (theRobotPose.invert() * Vector2<>(theFieldDimensions.xPosOwnGroundline, 0.f)).angle();
  }

  void LibCodeRelease::postProcess() {}

  bool LibCodeRelease::between(float value, float min, float max) { return value >= min && value <= max; }

  float LibCodeRelease::nearestPlayerInSight(float angle, float offset) {
    auto& players = thePlayerModel.players;
    auto minDistance = INFINITY;
    for (auto& player : players) {
      auto& localPos = player.relPosOnField;
      if (localPos.x <= 20) {
        continue;
      }

      if (std::abs(localPos.angle()) < angle && std::abs(localPos.y) < offset) {
        minDistance = std::min(minDistance, localPos.abs());
      }
    }
    return minDistance;
  }

  float LibCodeRelease::nearestOpponentInSight(float angle, float offset) {
    if (thePlayerModel.players.empty()) {
      return INFINITY;
    }

    std::vector<Vector2<>> players;
    for (const auto& player : thePlayerModel.players) {
      players.push_back(theRobotPose * player.relPosOnField);
    }
    std::vector<int> teamMateIDs = {1, 2, 3, 4, 5, 6, 7};
    teamMateIDs.erase(teamMateIDs.begin() + theRobotInfo.number - 1);
    std::vector<bool> playersAreTeamMates = isTeamMate(players, teamMateIDs);

    float minDistance = INFINITY;
    for (int i = 0; i < thePlayerModel.players.size(); i++) {
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

  std::vector<bool> LibCodeRelease::isTeamMate(std::vector<Vector2<>> players, const std::vector<int>& teamMateIDs) {
    std::vector<bool> isTeamMate(players.size(), false);
    const float tol = 200.f;
    for (const auto& id : teamMateIDs) {
      const Vector2<> teamMatePos = theTeamMateData.robotPoses[id].translation;
      float minDeviation = INFINITY;
      int minIndex = 0;

      for (int i = 0; i < players.size(); i++) {
        float deviation = (teamMatePos - players[i]).abs();
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

  bool LibCodeRelease::isAnyTeamMateActive(std::initializer_list<int> playerNums) {
    for (int playerNum : playerNums) {
      if (theTeamMateData.isActive[playerNum]) {
        return true;
      }
    }

    return false;
  }

  // Return true if an object of size "objectRadius" at position "pos" would touch any part of the penalty area
  // (100 mm is a robot foot size)
  bool LibCodeRelease::inGoalBox(const Vector2<> pos, float objectRadius = 100.f) {
    const float margin = theFieldDimensions.fieldLinesWidth * 0.5f + objectRadius;
    return pos.x <= theFieldDimensions.xPosOwnGoalBox + margin && std::abs(pos.y) <= theFieldDimensions.yPosLeftGoalBox;
  }

  // Number of active teammates (not including self, including fallen) currently in own penalty area
  int LibCodeRelease::numTeamMatesInGoalBox() {
    int count = 0;

    for (int i = 1; i < theTeamMateData.numOfPlayers; ++i) {
      if (i == theRobotInfo.number || !theTeamMateData.isActive[i]) {
        continue;
      }

      if (inGoalBox(theTeamMateData.robotPoses[i].translation)) {
        ++count;
      }
    }

    return count;
  }

  bool LibCodeRelease::nearCentre() {
    return std::abs(theRobotPose.translation.x) < 350 ||
           (std::abs(theRobotPose.translation.x) < 800 && std::abs(theRobotPose.translation.y) < 800);
  }

  std::list<Range<float>> LibCodeRelease::calculateFreeRanges(const Vector2<>& ball,
                                                              const Range<float>& goalRange,
                                                              const std::vector<int>& teamMateIDs) {
    // Represents a union of intervals of angles (defined in global frame centered on the ball)
    // that represent free regions when aiming at the goal
    std::list<Range<float>> freeRanges = {goalRange};
    if (thePlayerModel.players.size() == 0) {
      return freeRanges;
    }

    std::vector<Vector2<>> players;
    for (const auto& player : thePlayerModel.players) {
      players.push_back(theRobotPose * player.relPosOnField);
    }

    std::vector<bool> isTeamMate(players.size(), false);
    const float tol = 200.f; // TODO: find a reasonable tolerance
    for (const auto& id : teamMateIDs) {
      const Vector2<> teamMatePos = theTeamMateData.robotPoses[id].translation;
      float minDeviation = INFINITY;
      int minIndex = 0;

      for (int i = 0; i < players.size(); i++) {
        float deviation = (teamMatePos - players[i]).abs();
        if (deviation < minDeviation) {
          minDeviation = deviation;
          minIndex = i;
        }
      }

      if (minDeviation < tol) {
        isTeamMate[minIndex] = true;
      }
    }

    // Subtract a free interval for every player
    for (int i = 0; i < players.size(); i++) {
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

  Range<float> LibCodeRelease::getBestFreeRange(const std::list<Range<float>>& freeRanges, const float robotToBallAngle) {
    return *std::max_element(
      freeRanges.begin(), freeRanges.end(), [robotToBallAngle](const Range<float>& a, const Range<float>& b) {
        // Reward wide ranges, lightly penalize those far away
        const float rewardA = a.getSize() - 0.15f * std::abs(angleDifference(robotToBallAngle, a.getCenter()));
        const float rewardB = b.getSize() - 0.15f * std::abs(angleDifference(robotToBallAngle, b.getCenter()));
        return rewardA < rewardB;
      });
  }

  bool LibCodeRelease::teamMateInFreeRange(const std::list<Range<float>>& freeRanges,
                                           const Vector2<>& gloBall,
                                           const int teamMateID) {
    float ballToTeamMateAngle = (theTeamMateData.robotPoses[teamMateID].translation - gloBall).angle();
    for (const auto& fRange : freeRanges) {
      if (fRange.isInside(ballToTeamMateAngle)) {
        return true;
      }
    }
    return false;
  }

  int LibCodeRelease::getOtherActiveDefenders(int ownNumber) {
    int defenders = 0;
    for (int i = ownNumber + 1; i < TeamMateData::numOfPlayers; i++) {
      if ((theTeamMateData.behaviorStatus[i].role == BehaviorStatus::defender) &&
          (theTeamMateData.isActive[i])) { // the role of the robot itslef is dummy
        defenders += 1;
      }
    }
    return defenders;
  }

  Angle LibCodeRelease::computeBallDefendingDirection() {
    Vector2<> gloBallPos = theRobotPose * theBallModel.estimate.position;
    Vector2<> ownGoal2Ball = gloBallPos - Vector2<>(theFieldDimensions.xPosOwnGoal, theFieldDimensions.yPosCenterGoal);
    Vector2<> ball2OpponentGoal =
      Vector2<>(theFieldDimensions.xPosOpponentGoal, theFieldDimensions.yPosCenterGoal) - gloBallPos;
    float ratio = (ownGoal2Ball.abs() + 0.001f) / (ball2OpponentGoal.abs() + ownGoal2Ball.abs() + 0.001f);
    return Angle(ratio * ownGoal2Ball.angle() + (1.f - ratio) * ball2OpponentGoal.angle()).normalize();
  }

  bool
  LibCodeRelease::ballInsideAlignmentCircle(const Vector2<>& relBallPos, float aligmentCircleRadius, float aligmentOffset) {
    return (relBallPos - Vector2<>(aligmentOffset, 0.f)).abs() < aligmentCircleRadius;
  }

  bool LibCodeRelease::insidePatternCone(const Vector2<>& relBallPos, const float gloKickAngle, const float margin) {
    const float patternAlignmentDistance = 350.f * margin;
    const float patternAlignmentAngle = fromDegrees(25.f) * margin;
    if (relBallPos.abs() < patternAlignmentDistance) {
      return std::abs(gloKickAngle - (theRobotPose.rotation + computeAlignmentAngle(relBallPos))) < patternAlignmentAngle;
    } else
      return false;
  }

  Angle LibCodeRelease::computeAlignmentAngle(const Vector2<>& relBallPos) {
    const float alignmentOffsetY = 75.f;
    if (relBallPos.y > alignmentOffsetY) {
      return Vector2<>(relBallPos.x, relBallPos.y - alignmentOffsetY).angle();
    } else if (relBallPos.y < -alignmentOffsetY) {
      return Vector2<>(relBallPos.x, relBallPos.y + alignmentOffsetY).angle();
    } else {
      return Angle(0.f);
    }
  }

  Angle LibCodeRelease::computeRelBallTargetAngle(const Angle& gloBallTargetAngle) {
    Angle gloRobot2BallAngle =
      Angle(theRobotPose.rotation + computeAlignmentAngle(theBallModel.estimate.position)).normalize();
    return Angle(gloBallTargetAngle - gloRobot2BallAngle).normalize();
  }
} // namespace behavior
