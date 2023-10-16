/**
 * @file DynamicRoleAssigner.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Infrastructure/TeamInfo.h"

#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/GameInfo.h"

#include "Core/Debugging/Debugging.h"
#include "Core/Module/Module.h"
#include "Core/Range.h"
#include "Core/Math/Vector.h"

MODULE(DynamicRoleAssigner)
USES(RobotPose)
REQUIRES(FrameInfo)
REQUIRES(OwnTeamInfo)
REQUIRES(TeamMateData)
REQUIRES(RobotInfo)
REQUIRES(GameInfo)
PROVIDES_WITH_MODIFY(BehaviorStatus)
LOADS_PARAMETER(bool, useDynamicRoleAssignment)
END_MODULE

class DynamicRoleAssigner : public DynamicRoleAssignerBase {

  void reset(BehaviorStatus& behaviorStatus);
  void update(BehaviorStatus& behaviorStatus) override;

private:
  BehaviorConfig initBehaviorConfig;
  BehaviorStatus::Role currentRole;
  std::chrono::time_point<std::chrono::steady_clock> lastBeaviorSwitchTime;

  bool isInitialRun;

  typedef std::map<BehaviorStatus::Role, unsigned int> teamFormation;
  // Map which contains the teamFormation for a given number of active robots.
  std::map<unsigned int, teamFormation> teamFormationMap;

  /**
   * @brief Set role to defender if the robot has been penalized.
   *
   * @param behaviorStatus representation in which to write the role
   */
  void setPenalizedPlayerAsDefender(BehaviorStatus& behaviorStatus);

  /**
   * @brief Updates the BehaviorStatus to the new role and takes care of modifying the state of
   * the DynamicRoleAssigner class.
   *
   * @param behaviorStatus representation where role is updated
   * @param newRole new role which the robot should take
   */
  void updateRole(BehaviorStatus& behaviorStatus, const BehaviorStatus::Role newRole);

  /**
   * @brief Set up the teamFormationMap
   */
  void setTeamFormation();

  /**
   * @brief promotes one of the defenders to a striker if there are too many defenders.
   *
   * @param behaviorStatus representation where role is updated
   */
  void promotePlayersToRole(BehaviorStatus& behaviorStatus,
                            const BehaviorStatus::Role currentRole,
                            const BehaviorStatus::Role updateRole);

  /**
   * @brief get the number of players in the team with the given role
   *
   * @param role for which we wish to know the number of players
   * @return vector containing the player ids with the specified role
   */
  std::vector<TeamMateData::PlayerNum> getPlayersWithRole(const BehaviorStatus::Role role);

  /**
   * @brief get the player id of the most advanced player in the specified player ids
   *
   * @param playerIds vector of player ids
   * @return playernum of the most forward player
   */
  TeamMateData::PlayerNum getMostForwardPlayer(const std::vector<TeamMateData::PlayerNum>& playerIds);

  /**
   * @brief reset playerrole if number of strikers exceeds the amount in the formation if the player is
   * currently a striker
   *
   * @param behaviorStatus representation where role is updated
   */
  void resetStrikers(BehaviorStatus& behaviorStatus);

public:
  DynamicRoleAssigner();
};
