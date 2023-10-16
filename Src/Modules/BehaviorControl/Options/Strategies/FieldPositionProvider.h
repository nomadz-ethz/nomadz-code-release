/**
 * @file FieldPositionProvider.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Modeling/PlayerModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/FieldPosition.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"

#include "Core/Debugging/Debugging.h"
#include "Core/Module/Module.h"
#include "Core/Range.h"
#include "Core/Math/Vector.h"

MODULE(FieldPositionProvider)
USES(RobotPose)
REQUIRES(FrameInfo)
REQUIRES(GameInfo)
REQUIRES(BehaviorStatus)
REQUIRES(FieldDimensions)
REQUIRES(TeamMateData)
REQUIRES(RobotInfo)
REQUIRES(OwnTeamInfo)
REQUIRES(BallModel)
REQUIRES(CombinedWorldModel)
REQUIRES(PlayerModel)
PROVIDES_WITH_MODIFY(FieldPosition)
LOADS_PARAMETER(Vector2<>, setPosKeeper)

LOADS_PARAMETER(Vector2<>, setPosStriker)
// LOADS_PARAMETER(vector<Vector2<>>, setPosStrikerList)
// std::vector<Vector2<>> setPosStrikerList
// {setPosStriker,setPosStrikerOne,setPosStrikerTwo,setPosStrikerThree,setPosStrikerFour};
LOADS_PARAMETER(Vector2<>, setPosSupporter)
LOADS_PARAMETER(Vector2<>, setPosDefender)
LOADS_PARAMETER(Vector2<>, setPosDefenderOne)
LOADS_PARAMETER(Vector2<>, setPosDefenderTwo)
LOADS_PARAMETER(Vector2<>, setPosOther)
LOADS_PARAMETER(Vector2<>, setPosKickoffKeeper)

LOADS_PARAMETER(Vector2<>, setPosKickoffStriker)
LOADS_PARAMETER(Vector2<>, setPosKickoffSupporter)
LOADS_PARAMETER(Vector2<>, setPosKickoffDefender)
LOADS_PARAMETER(Vector2<>, setPosKickoffDefenderOne)
LOADS_PARAMETER(Vector2<>, setPosKickoffDefenderTwo)
LOADS_PARAMETER(Vector2<>, setPosSupporterTargetOne)
LOADS_PARAMETER(Vector2<>, setPosSupporterTargetTwo)
LOADS_PARAMETER(Vector2<>, setPosStrikerTargetOne)
LOADS_PARAMETER(Vector2<>, setPosStrikerTargetTwo)
LOADS_PARAMETER(Vector2<>, setPosKickoffOther)

LOADS_PARAMETER(Vector2<>, posGoalFreeKickStrikerOne)
LOADS_PARAMETER(Vector2<>, posGoalFreeKickStrikerTwo)
LOADS_PARAMETER(Vector2<>, posGoalFreeKickSupporter)
LOADS_PARAMETER(Vector2<>, posGoalFreeKickDefenderEven)
LOADS_PARAMETER(Vector2<>, posGoalFreeKickDefenderOdd)

LOADS_PARAMETER(Vector2<>, posCornerFreeKickStriker)
LOADS_PARAMETER(float, rotKickInStriker)
LOADS_PARAMETER(Vector2<>, posKickInStriker)
LOADS_PARAMETER(float, rotCornerFreeKickStriker)
LOADS_PARAMETER(Vector2<>, posCornerFreeKickSupporter)
LOADS_PARAMETER(float, rotKickInSupporter)
LOADS_PARAMETER(Vector2<>, posKickInSupporter)
LOADS_PARAMETER(float, rotCornerFreeKickSupporter)
LOADS_PARAMETER(Vector2<>, posCornerFreeKickDefenderEven)
LOADS_PARAMETER(Vector2<>, posCornerFreeKickDefenderOdd)

LOADS_PARAMETER(Vector2<>, setPosPenaltyKeeper)
LOADS_PARAMETER(Vector2<>, setPosPenaltyStriker)
LOADS_PARAMETER(Vector2<>, setPosPenaltySupporter)
LOADS_PARAMETER(Vector2<>, setPosPenaltyDefenderEven)
LOADS_PARAMETER(Vector2<>, setPosPenaltyDefenderOdd)
LOADS_PARAMETER(float, roleReassignPeriod)

LOADS_PARAMETER(Vector2<>, penaltyStrikerDx)
LOADS_PARAMETER(Vector2<>, penaltyStrikerDy)
LOADS_PARAMETER(Vector2<>, penaltySupporterDx)
LOADS_PARAMETER(Vector2<>, penaltySupporterDy)

LOADS_PARAMETER(Vector2<>, setPosStrikerOne)
LOADS_PARAMETER(Vector2<>, setPosStrikerTwo)
LOADS_PARAMETER(Vector2<>, setPosStrikerThree)
LOADS_PARAMETER(Vector2<>, setPosStrikerFour)

LOADS_PARAMETER(Vector2<>, setPosKickoffStrikerOne)
LOADS_PARAMETER(Vector2<>, setPosKickoffStrikerTwo)
LOADS_PARAMETER(Vector2<>, setPosKickoffStrikerThree)
LOADS_PARAMETER(Vector2<>, setPosKickoffStrikerFour)

LOADS_PARAMETER(Vector2<>, setPosSupporterOne)
LOADS_PARAMETER(Vector2<>, setPosSupporterTwo)
LOADS_PARAMETER(Vector2<>, setPosSupporterThree)
LOADS_PARAMETER(Vector2<>, setPosSupporterFour)

LOADS_PARAMETER(Vector2<>, setPosKickoffSupporterOne)
LOADS_PARAMETER(Vector2<>, setPosKickoffSupporterTwo)
LOADS_PARAMETER(Vector2<>, setPosKickoffSupporterThree)
LOADS_PARAMETER(Vector2<>, setPosKickoffSupporterFour)

LOADS_PARAMETER(bool, useOptimalPosition)
END_MODULE

class FieldPositionProvider : public FieldPositionProviderBase {

private:
  void update(FieldPosition& fieldPosition) override;
  Pose2D denormalizePose(const Pose2D& pose);
  Pose2D normalizePose(const Pose2D& pose);
  Pose2D getReadyPose();
  Pose2D getStandByPose(FieldPosition& fieldPosition);

  /*
   * Optimizes over cost functions to provide the best motion direction with GD.
   * The cost is computed in 9 positions around the robot, and the lower gradient is used to compute the
   * next target.
   * The cost accounts for: field coverage, CoM of the formation, detected obstacles.
   * They are tuned considering positions in meters.
   * Currently is tuned to be deployed only on offensive players
   *
   * @return an intermediate setpoint for each offensive player
   */
  Pose2D getCurrentActivePose();

  /* Provides the pose robots would reach while searching for the ball, if localized.
   * Using !optimal, robots would positions on 2 ellipses, one for offensive players and one for defensive,
   * evenly spaced. They actively account for penalized teammates.
   *
   * @param optimal : solve the optimal problem with GD
   * @return the search pose for each robot
   */
  Pose2D getSearchPose(bool optimal);

  /*
   * Checks how many teammates of the given role are not penalized.
   * Cannot check for active robots since during Ready state they don't send
   * messages, hence resulting to be inactive
   *
   * @param ownNumber : consider only robots with number larger than this.
   * Use theRobotInfo.number to get a different result for each robot, useful for
   * positioning in a trajectory.
   * Use 0 to get all robots (-1 since the robot itself is a dummy variable)
   * @return the number of non penalized robots
   */
  int getOtherActiveStrikers(int ownNumber);
  int getOtherActiveSupporters(int ownNumber);
  int getOtherActiveDefenders(int ownNumber);

  Pose2D assignStrikerPosition(int otherActStriker);
  Pose2D assignKickoffStrikerPosition(int otherActStriker, int actSupporter);
  Pose2D assignSupporterPosition(int otherActStriker);
  Pose2D assignKickoffSupporterPosition(int otherActStriker);
  Pose2D assignDefenderPosition(int otherActDefender);
  Pose2D assignKickoffDefenderPosition(int otherActDefender);
  Pose2D penaltyStrikerPosition(float rot, int otherStriker);
  Pose2D penaltySupporterPosition(float rot, int otherSupporter);

  /* Computes a positive penalty gaussian-shaped if the robot approaches
   * an obstacle in its field of view
   *
   * @param ownPos : the position wrt the cost is computed
   * @return cost
   */
  float avoidanceGaussian(Vector2<> ownPos);

  /* Each robot is modeled with a Gaussian, they aim to space in the field
   * to optimize the integral of the difference between the functions of teammates.
   * TeamMateData is used, so delay is present.
   *
   * @param ownPos : the position wrt the cost is computed
   * @return cost
   */
  float coverageIntegral(Vector2<> ownPos);

  /* The baricenter of the offensive players formation is moved towards the
   * global ball location with a quadratic cost. Uses delayed positions.
   *
   * @param ownPos : the position wrt the cost is computed
   * @return cost
   */
  float ballCoM(Vector2<> ownPos);
  float awayFromGoal(Vector2<> ownPos);

  Pose2D getDefenderStandByPose();

public:
  FieldPositionProvider();
};
