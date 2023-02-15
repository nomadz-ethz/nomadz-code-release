/**
 * @file ZMPIPControllerNGModule.h
 *
 * Module wrapper for the ZMP/IP-Controller
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Core/Module/Module.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/RefZMP.h"
#include "Representations/Sensing/ZMPModel.h"
#include "ZMPIPControllerNG.h"
#include "Representations/MotionControl/PatternGenRequest.h"

MODULE(ZMPIPControllerNGModule)
REQUIRES(RobotModel)
REQUIRES(InertiaSensorData)
REQUIRES(SensorData)
REQUIRES(WalkingEngineParams)
REQUIRES(FallDownState)
REQUIRES(ActualCoM)
REQUIRES(PatternGenRequest)
REQUIRES(ControllerParams)
REQUIRES(ZMPModel)
REQUIRES(RefZMP)
USES(WalkingInfo)
PROVIDES_WITH_MODIFY(TargetCoM)
END_MODULE

class ZMPIPControllerNGModule : public ZMPIPControllerNGModuleBase {
public:
  ZMPIPControllerNGModule();

  ZMPIPControllerNG controller;

  void update(TargetCoM& theTargetCoM);
};
