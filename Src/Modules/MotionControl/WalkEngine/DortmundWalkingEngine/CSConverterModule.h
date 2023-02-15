/**
 * @file CSConverterModule.h
 *
 * Module wrapper for the CSConverter
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */
#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Core/Module/Module.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/BodyTilt.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/InertiaSensorData.h"
//#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "CSConverter.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/ArmContact.h"
#include "Representations/Configuration/RobotDimensions.h"

MODULE(CSConverterModule)
REQUIRES(TargetCoM)
REQUIRES(WalkingEngineParams)
REQUIRES(ControllerParams)
REQUIRES(ActualCoMRCS)
REQUIRES(BodyTilt)
REQUIRES(TorsoMatrix)
REQUIRES(RobotModel)
REQUIRES(RobotInfo)
// REQUIRES(FsrSensorData)
REQUIRES(InertiaSensorData)
REQUIRES(FallDownState)
REQUIRES(Footpositions)
REQUIRES(FootSteps)
REQUIRES(ArmContact)
REQUIRES(RobotDimensions)
REQUIRES(WalkCalibration)
PROVIDES_WITH_MODIFY(KinematicRequest)
PROVIDES_WITH_MODIFY(WalkingInfo)
PROVIDES_WITH_MODIFY(FixedOdometryRobotPose)
END_MODULE

class CSConverterModule : public CSConverterModuleBase {
public:
  CSConverterModule();

  CSConverter converter;

  void update(KinematicRequest& kinematicRequest);
  void update(WalkingInfo& walkingInfo);
  void update(FixedOdometryRobotPose& fixedOdometryRobotPose);
};
