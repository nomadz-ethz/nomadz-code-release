/**
 * @file LibWorldModel.cpp
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "../LibraryBase.h"

#include "Representations/Modeling/CombinedWorldModel.h"

namespace behavior {
#include "LibWorldModel.h"

  LibWorldModel::LibWorldModel() {}

  void LibWorldModel::preProcess() {}

  void LibWorldModel::postProcess() {}

  // Returns if there is a valid kick estimate position
  Vector2<> LibWorldModel::kickEstimate(float maxTime) {
    Vector2<> estimate = {NAN, NAN};
    for (unsigned i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i) {
      float timeSinceKick = theFrameInfo.getTimeSince(theTeamMateData.robotsPersonalData[i].lastKickTime);
      if (timeSinceKick < maxTime) {
        maxTime = timeSinceKick;
        estimate = theTeamMateData.robotsPersonalData[i].kickEstimate;
      }
    }
    return estimate;
  }

  // Gives the last time a ball was seen, either by the combined world model or the player itself
  float LibWorldModel::timeSinceBallLastSeen() {
    return std::min(theBallModel.timeSinceLastSeen, theCombinedWorldModel.timeSinceBallLastSeenOthers);
  }
  // Gives the local perception if available and otherwise the world model
  Vector2<> LibWorldModel::ballPosition(float localSeen) {
    if (theBallModel.timeSinceLastSeen < localSeen) {
      return theRobotPose * theBallModel.estimate.position;
    } else {
      return theCombinedWorldModel.ballStateOthers.position;
    }
  }

  // Nearest player in sight in world model (usually you want to use only the local perceptions)
  float LibWorldModel::nearestPlayerInSight(float angle) {
    std::vector<Pose2D> players = theCombinedWorldModel.positionsOwnTeam;
    auto& opponents = theCombinedWorldModel.positionsOpponentTeam;
    for (auto& opponent : opponents) {
      players.push_back(opponent.robotPosition);
    }
    auto minDistance = INFINITY;
    for (auto& player : players) {
      auto localPos = theRobotPoseAfterPreview.invert() * player.translation;
      if (localPos.x <= 20) {
        continue;
      }

      if (std::abs(localPos.angle()) < angle) {
        minDistance = std::min(minDistance, localPos.abs());
      }
    }
    return minDistance;
  }
} // namespace behavior
