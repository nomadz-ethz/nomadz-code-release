/**
 * @file DynamicRoleAssigner.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include "Representations/BehaviorControl/BehaviorConfig.h"

#include "DynamicRoleAssigner.h"

MAKE_MODULE(DynamicRoleAssigner, Behavior Control);

DynamicRoleAssigner::DynamicRoleAssigner() {
  InMapFile stream("RobotBehaviorConfig.cfg");
  ASSERT(stream.exists());
  stream >> initBehaviorConfig;

  currentRole = BehaviorStatus::Role::dummy;
  isInitialRun = true;
  lastBeaviorSwitchTime = std::chrono::steady_clock::now() - std::chrono::hours(24);

  setTeamFormation();
}

void DynamicRoleAssigner::update(BehaviorStatus& behaviorStatus) {
  MODIFY("parameters:robotBehavior", initBehaviorConfig);

  behaviorStatus.roleChanged = false;
  behaviorStatus.lost = theRobotPose.lost == false ? BehaviorStatus::Lost::no : BehaviorStatus::Lost::yes;

  // Set up initial behavior during initial call to this function.
  if (isInitialRun) {
    isInitialRun = false;
    behaviorStatus.teamColor = theOwnTeamInfo.teamColor == TEAM_BLUE ? BehaviorStatus::blue : BehaviorStatus::red;
    updateRole(behaviorStatus, initBehaviorConfig.role);
    return;
  }

  // If we are not playing or the module has been disabled, we do not modify the roles.
  if (theGameInfo.state != STATE_PLAYING || !useDynamicRoleAssignment) {
    return;
  }

  // Set role as defender if the robot is penalized.
  setPenalizedPlayerAsDefender(behaviorStatus);
  if (behaviorStatus.roleChanged) {
    return;
  }

  promotePlayersToRole(behaviorStatus, BehaviorStatus::Role::defender, BehaviorStatus::Role::supporter);
  if (behaviorStatus.roleChanged) {
    return;
  }

  promotePlayersToRole(behaviorStatus, BehaviorStatus::Role::supporter, BehaviorStatus::Role::striker);
  if (behaviorStatus.roleChanged) {
    return;
  }

  resetStrikers(behaviorStatus);
  if (behaviorStatus.roleChanged) {
    return;
  }
}

void DynamicRoleAssigner::setPenalizedPlayerAsDefender(BehaviorStatus& behaviorStatus) {
  if (initBehaviorConfig.role == BehaviorStatus::Role::keeper) {
    return;
  }

  if (theRobotInfo.penalty != 0 && currentRole != BehaviorStatus::Role::defender) {
    updateRole(behaviorStatus, BehaviorStatus::Role::defender);
  }
}

void DynamicRoleAssigner::promotePlayersToRole(BehaviorStatus& behaviorStatus,
                                               const BehaviorStatus::Role currentPlayerRole,
                                               const BehaviorStatus::Role newPlayerRole) {
  const unsigned int numPlayers = theTeamMateData.numOfConnectedTeamMates + 1;
  // count number of active defenders
  const auto playerIds = getPlayersWithRole(currentPlayerRole);

  if (playerIds.size() <= teamFormationMap[numPlayers][currentPlayerRole]) {
    return;
  }

  TeamMateData::PlayerNum mostForwardPlayer = getMostForwardPlayer(playerIds);

  if (mostForwardPlayer == static_cast<TeamMateData::PlayerNum>(static_cast<int>(theRobotInfo.number))) {
    updateRole(behaviorStatus, newPlayerRole);
  }
}

std::vector<TeamMateData::PlayerNum> DynamicRoleAssigner::getPlayersWithRole(const BehaviorStatus::Role role) {
  std::vector<TeamMateData::PlayerNum> filtered_players;

  for (uint playerNum = TeamMateData::firstPlayer; playerNum < TeamMateData::numOfPlayers; ++playerNum) {
    if (theTeamMateData.behaviorStatus[playerNum].role == role && !theTeamMateData.isPenalized[playerNum]) {
      filtered_players.push_back(static_cast<TeamMateData::PlayerNum>(playerNum));
    }
  }

  // Personal info is not stored in theTeamMateData
  if (currentRole == role && theRobotInfo.penalty == 0) {
    filtered_players.push_back(static_cast<TeamMateData::PlayerNum>(static_cast<int>(theRobotInfo.number)));
  }

  return filtered_players;
}

TeamMateData::PlayerNum DynamicRoleAssigner::getMostForwardPlayer(const std::vector<TeamMateData::PlayerNum>& playerIds) {
  float mostForwardPosition = -std::numeric_limits<float>::max();
  TeamMateData::PlayerNum mostForwardPlayer = TeamMateData::PlayerNum::noPlayer;

  for (auto idx : playerIds) {
    if (idx == theRobotInfo.number) {
      continue;
    }

    const RobotPose currentRobotPose = theTeamMateData.robotPoses[idx];

    // NOTE(@naefjo): We don't need to check for validity since invalid data is not even sent.
    if (currentRobotPose.translation.x > mostForwardPosition) {
      mostForwardPosition = currentRobotPose.translation.x;
      mostForwardPlayer = idx;
    }
  }

  if (theRobotPose.translation.x > mostForwardPosition && theRobotPose.valid && !theRobotPose.lost) {
    mostForwardPlayer = static_cast<TeamMateData::PlayerNum>(static_cast<int>(theRobotInfo.number));
  }

  return mostForwardPlayer;
}

void DynamicRoleAssigner::resetStrikers(BehaviorStatus& behaviorStatus) {
  const auto playerIds = getPlayersWithRole(BehaviorStatus::Role::striker);
  const unsigned int numPlayers = theTeamMateData.numOfConnectedTeamMates + 1;

  if (playerIds.size() > teamFormationMap[numPlayers][BehaviorStatus::Role::striker] &&
      currentRole != initBehaviorConfig.role && currentRole == BehaviorStatus::Role::striker) {
    updateRole(behaviorStatus, initBehaviorConfig.role);
  }
}

void DynamicRoleAssigner::updateRole(BehaviorStatus& behaviorStatus, const BehaviorStatus::Role newRole) {
  auto currentTime = std::chrono::steady_clock::now();
  // NOTE(@naefjo): Hard coded hysteresis of 5s to prevent rapid behavior switching.
  if (currentTime - lastBeaviorSwitchTime > std::chrono::seconds(5) && currentRole != newRole) {
    behaviorStatus.role = newRole;
    behaviorStatus.roleChanged = true;
    currentRole = newRole;
    lastBeaviorSwitchTime = currentTime;
  }
}

void DynamicRoleAssigner::setTeamFormation() {
  teamFormation numPlayers1{{BehaviorStatus::Role::striker, 1}};
  teamFormation numPlayers2{{BehaviorStatus::Role::defender, 1}, {BehaviorStatus::Role::striker, 1}};
  teamFormation numPlayers3{
    {BehaviorStatus::Role::keeper, 1}, {BehaviorStatus::Role::defender, 1}, {BehaviorStatus::Role::striker, 1}};
  teamFormation numPlayers4{
    {BehaviorStatus::Role::keeper, 1}, {BehaviorStatus::Role::defender, 2}, {BehaviorStatus::Role::striker, 1}};
  teamFormation numPlayers5{{BehaviorStatus::Role::keeper, 1},
                            {BehaviorStatus::Role::defender, 2},
                            {BehaviorStatus::Role::supporter, 1},
                            {BehaviorStatus::Role::striker, 1}};
  teamFormation numPlayers6{{BehaviorStatus::Role::keeper, 1},
                            {BehaviorStatus::Role::defender, 2},
                            {BehaviorStatus::Role::supporter, 1},
                            {BehaviorStatus::Role::striker, 2}};
  teamFormation numPlayers7{{BehaviorStatus::Role::keeper, 1},
                            {BehaviorStatus::Role::defender, 2},
                            {BehaviorStatus::Role::supporter, 2},
                            {BehaviorStatus::Role::striker, 2}};

  teamFormationMap = {{1, numPlayers1},
                      {2, numPlayers2},
                      {3, numPlayers3},
                      {4, numPlayers4},
                      {5, numPlayers5},
                      {6, numPlayers6},
                      {7, numPlayers7}};
}