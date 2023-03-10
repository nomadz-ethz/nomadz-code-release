/**
 * @file TeamDataSender.h
 *
 * Declaration of module TeamDataSender
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are Colin Graf
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/LineAnalysis.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/Infrastructure/PersonalData.h"
#include "Representations/Infrastructure/TeamInfo.h"

MODULE(TeamDataSender)
REQUIRES(FrameInfo)
REQUIRES(TeamMateData)
REQUIRES(RobotPose)
REQUIRES(SideConfidence)
REQUIRES(BallModel)
REQUIRES(PlayerModel)
REQUIRES(MotionRequest)
REQUIRES(FilteredSensorData)
REQUIRES(JointRequest)
REQUIRES(RobotHealth)
REQUIRES(RobotInfo)
REQUIRES(GroundContactState)
REQUIRES(FallDownState)
REQUIRES(CombinedWorldModel)
REQUIRES(CameraMatrix)
REQUIRES(LineAnalysis)
REQUIRES(PersonalData) // Personal Data
REQUIRES(OwnTeamInfo)
REQUIRES(Whistle)
REQUIRES(BehaviorControlOutput)
PROVIDES(TeamDataSenderOutput)
LOADS_PARAMETER(unsigned int, maxNumberOfRobotsToSend)           /**< Do not send more robots than this. */
LOADS_PARAMETER(unsigned int, maxNumberOfObstaclesToSend)        /**< Do not send more obstacles than this. */
LOADS_PARAMETER(unsigned int, maxNumberOfObstacleClustersToSend) /**< Do not send more obstacle clusters than this. */
END_MODULE

/**
 * @class TeamDataSender
 * A modules for sending some representation to teammates
 */
class TeamDataSender : public TeamDataSenderBase {
public:
  /** Default constructor */
  TeamDataSender() : TeamDataSenderBase("teamDataSender.cfg"), sendFrames(0) {}

private:
  unsigned int sendFrames; /** Quantity of frames in which team data was sent */
                           /**
                            * The update function called in each cognition process cycle
                            * @param teamDataSenderOutput An empty output representation
                            */
  virtual void update(TeamDataSenderOutput& teamDataSenderOutput);
};
