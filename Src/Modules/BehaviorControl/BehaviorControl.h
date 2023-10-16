/**
 * @file BehaviorControl.h
 *
 * Declaration of the base class of the C-based state machine behavior control module.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
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
#include "Representations/BehaviorControl/FieldPosition.h"
#include "Representations/BehaviorControl/LocalSkill.h"
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
#include "Representations/MotionControl/SpecialActionsOutput.h"
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
#include "Representations/MotionControl/PlannedSteps.h"
#include <limits>
#include <algorithm>

MODULE(BehaviorControl)
REQUIRES(SpecialActionsOutput)
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
REQUIRES(BehaviorStatus)
REQUIRES(FieldPosition)
REQUIRES(PersonalData)
REQUIRES(PlannedSteps)
REQUIRES(LocalSkill)

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
LOADS_PARAMETER(Vector2<>, kickOffsetOmniKickLong)
LOADS_PARAMETER(float, switchThreshold)
LOADS_PARAMETER(bool, optimizeStrategy)
LOADS_PARAMETER(std::string, refereeSide)
LOADS_PARAMETER(bool, useRefereeChallenge)
LOADS_PARAMETER(float, friction)
LOADS_PARAMETER(float, playerAvoidanceRadius)
LOADS_PARAMETER(Rangef, targetDistanceRange)
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

  public:
    ArmMotionRequest& theArmMotionRequest;
    MotionRequest& theMotionRequest;
    HeadMotionRequest& theHeadMotionRequest; /**< The head motion request that will be set. */
    GameInfo& theGameInfo;                   /**< The game info that might be modified through button clicks. */
    RobotInfo& theRobotInfo;                 /**< The robot info that might be modified through button clicks. */
    OwnTeamInfo& theOwnTeamInfo;             /**< The own team info that might be modified through button clicks. */
    ActivationGraph& theActivationGraph;
    BehaviorLEDRequest& theBehaviorLEDRequest;

    BehaviorBase(internal::ref_init_tag t)
        : BehaviorControlBase(t), theArmMotionRequest(theArmMotionRequest), theMotionRequest(theMotionRequest),
          theHeadMotionRequest(theHeadMotionRequest), theGameInfo(theGameInfo), theRobotInfo(theRobotInfo),
          theOwnTeamInfo(theOwnTeamInfo), theActivationGraph(theActivationGraph),
          theBehaviorLEDRequest(theBehaviorLEDRequest) {}

    BehaviorBase(const BehaviorControlBase& base, BehaviorControlOutput& behaviorControlOutput)
        : BehaviorControlBase(base), theArmMotionRequest(behaviorControlOutput.armMotionRequest),
          theMotionRequest(behaviorControlOutput.motionRequest),
          theHeadMotionRequest(behaviorControlOutput.headMotionRequest), theGameInfo(behaviorControlOutput.gameInfo),
          theRobotInfo(behaviorControlOutput.robotInfo), theOwnTeamInfo(behaviorControlOutput.ownTeamInfo),
          theActivationGraph(behaviorControlOutput.executionGraph),
          theBehaviorLEDRequest(behaviorControlOutput.behaviorLEDRequest) {}
  };
} // namespace behavior
