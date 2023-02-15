/**
 * @file BehaviorControl.h
 *
 * Declaration of the base class of the C-based state machine behavior control module.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are Thomas Röfer Tim Laue
 */

#pragma once
#include <sstream>

#include "Core/Math/Common.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"
#include "Core/Module/Module.h"
#include "Core/Debugging/Debugging.h"
#include "Core/Math/GaussianDistribution2D.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/KeyStates.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/PlayerModel.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/MotionControl/ArmMotionEngineOutput.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/RefereePercept.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootContactModel.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Infrastructure/PersonalData.h"
#include "Representations/Modeling/PassHelper.h"
#include "Representations/Modeling/CoordinatedPassRepresentation.h"

#include <limits>
#include <algorithm>

MODULE(BehaviorControl)
REQUIRES(ArmContactModel)
REQUIRES(ArmMotionEngineOutput)
REQUIRES(BallModel)
REQUIRES(BallModelAfterPreview)
REQUIRES(KickEngineOutput)
// REQUIRES(CameraInfo)
REQUIRES(CameraMatrix)
REQUIRES(CombinedWorldModel)
REQUIRES(FallDownState)
REQUIRES(FrameInfo)
REQUIRES(FieldDimensions)
REQUIRES(GameInfo)
REQUIRES(GoalPercept)
REQUIRES(GroundContactState)
REQUIRES(HeadJointRequest)
REQUIRES(HeadLimits)
REQUIRES(JointData)
REQUIRES(KeyStates)
REQUIRES(MotionInfo)
REQUIRES(OwnTeamInfo)
REQUIRES(OpponentTeamInfo)
REQUIRES(PlayerModel)
REQUIRES(RefereePercept)
REQUIRES(RobotDimensions)
REQUIRES(RobotInfo)
REQUIRES(RobotPose)
REQUIRES(RobotPoseAfterPreview)
REQUIRES(SensorData)
REQUIRES(Whistle)
REQUIRES(FootContactModel)
REQUIRES(TeamMateData)
REQUIRES(TorsoMatrix) // Required for bike kicks
                      //  REQUIRES(PassHelper)
                      //  REQUIRES(CoordinatedPassRepresentation)

PROVIDES_WITH_OUTPUT(PersonalData) // Personal Data
PROVIDES_WITH_MODIFY_AND_OUTPUT(BehaviorControlOutput)
REQUIRES(BehaviorControlOutput)
PROVIDES_WITH_MODIFY(ArmMotionRequest)
PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(MotionRequest)
PROVIDES_WITH_MODIFY(HeadMotionRequest)
PROVIDES_WITH_MODIFY_AND_OUTPUT(ActivationGraph)
PROVIDES_WITH_MODIFY(BehaviorLEDRequest)
LOADS_PARAMETER(Vector2<>, kickOffsetLeftFieldSide)
LOADS_PARAMETER(Vector2<>, kickOffsetRightFieldSide)
LOADS_PARAMETER(Vector2<>, kickOffsetMiddleOfField)
LOADS_PARAMETER(Vector2<>, kickOffsetPass)
LOADS_PARAMETER(Vector2<>, kickOffsetOmniKick)
LOADS_PARAMETER(Vector2<>, kickOffsetOmniKickLong);
LOADS_PARAMETER(float, switchThreshold);
LOADS_PARAMETER(Vector2<>, setPosKeeper)
LOADS_PARAMETER(Vector2<>, setPosStriker)
LOADS_PARAMETER(Vector2<>, setPosSupporter)
LOADS_PARAMETER(Vector2<>, setPosDefenderEven)
LOADS_PARAMETER(Vector2<>, setPosDefenderOdd)
LOADS_PARAMETER(Vector2<>, setPosOther)
LOADS_PARAMETER(Vector2<>, setPosKickoffKeeper)
LOADS_PARAMETER(Vector2<>, setPosKickoffStriker)
LOADS_PARAMETER(Vector2<>, setPosKickoffSupporter)
LOADS_PARAMETER(Vector2<>, setPosKickoffDefenderEven)
LOADS_PARAMETER(Vector2<>, setPosKickoffDefenderOdd)
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

LOADS_PARAMETER(float, ballLockDistBarrier)

LOADS_PARAMETER(bool, optimizeStrategy)

LOADS_PARAMETER(float, playerAvoidanceRadius)
LOADS_PARAMETER(bool, supporterCatchBall)
LOADS_PARAMETER(Rangef, targetAngleRange)
LOADS_PARAMETER(Rangef, targetDistanceRange)
LOADS_PARAMETER(float, ballValidTime)
LOADS_PARAMETER(float, ballLostTime)
LOADS_PARAMETER(float, robotPoseValidTime)
LOADS_PARAMETER(float, robotPoseLostTime)
END_MODULE

namespace behavior {
  /**
   * Common base class for behavior options and libraries.
   */
  class BehaviorBase : public BehaviorControlBase {
  private:
    void update(BehaviorControlOutput& behaviorControlOutput) {}
    void update(MotionRequest& motionRequest) {}
    void update(ArmMotionRequest& armMotionRequest) {}
    void update(HeadMotionRequest& headMotionRequest) {}
    void update(BehaviorLEDRequest& behaviorLEDRequest) {}
    void update(ActivationGraph& executionGraph) {}
    void update(PersonalData& personalData) {} // Personal Data

  public:
    BehaviorStatus& theBehaviorStatus;
    ArmMotionRequest& theArmMotionRequest;
    MotionRequest& theMotionRequest;
    HeadMotionRequest& theHeadMotionRequest; /**< The head motion request that will be set. */
    GameInfo& theGameInfo;                   /**< The game info that might be modified through button clicks. */
    RobotInfo& theRobotInfo;                 /**< The robot info that might be modified through button clicks. */
    OwnTeamInfo& theOwnTeamInfo;             /**< The own team info that might be modified through button clicks. */
    ActivationGraph& theActivationGraph;
    PersonalData& thePersonalData;
    BehaviorLEDRequest& theBehaviorLEDRequest;

    BehaviorBase(internal::ref_init_tag t)
        : BehaviorControlBase(t), theBehaviorStatus(theBehaviorStatus), theArmMotionRequest(theArmMotionRequest),
          theMotionRequest(theMotionRequest), theHeadMotionRequest(theHeadMotionRequest), theGameInfo(theGameInfo),
          theRobotInfo(theRobotInfo), theOwnTeamInfo(theOwnTeamInfo), theActivationGraph(theActivationGraph),
          thePersonalData(thePersonalData), theBehaviorLEDRequest(theBehaviorLEDRequest) {}

    BehaviorBase(const BehaviorControlBase& base, BehaviorControlOutput& behaviorControlOutput)
        : BehaviorControlBase(base), theBehaviorStatus(behaviorControlOutput.behaviorStatus),
          theArmMotionRequest(behaviorControlOutput.armMotionRequest), theMotionRequest(behaviorControlOutput.motionRequest),
          theHeadMotionRequest(behaviorControlOutput.headMotionRequest), theGameInfo(behaviorControlOutput.gameInfo),
          theRobotInfo(behaviorControlOutput.robotInfo), theOwnTeamInfo(behaviorControlOutput.ownTeamInfo),
          theActivationGraph(behaviorControlOutput.executionGraph), thePersonalData(behaviorControlOutput.personalData),
          theBehaviorLEDRequest(behaviorControlOutput.behaviorLEDRequest) {}
  };
} // namespace behavior
