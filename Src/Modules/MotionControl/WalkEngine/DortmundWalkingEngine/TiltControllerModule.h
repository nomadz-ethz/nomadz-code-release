/**
 * @file TiltControllerModule.h
 *
 * Module wrapper for the TiltController
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Core/Module/Module.h"
#include "TiltController.h"
#include "Representations/Sensing/InertiaSensorData.h"
//#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/BodyTilt.h"
//#include "Representations/MotionControl/SpeedInfo.h"
//#include "Representations/MotionControl/TargetCoM.h"
//#include "Representations/Configuration/JointCalibration.h"

MODULE(TiltControllerModule)
REQUIRES(InertiaSensorData)
// USES(JointRequest),
// REQUIRES(JointCalibration),
REQUIRES(WalkingEngineParams)
// REQUIRES(SpeedInfo),
// REQUIRES(TargetCoM),
PROVIDES_WITH_MODIFY(BodyTilt)
END_MODULE

class TiltControllerModule : public TiltControllerModuleBase {
public:
  TiltControllerModule();

  TiltController controller;

  void update(BodyTilt& theBodyTilt);
};
