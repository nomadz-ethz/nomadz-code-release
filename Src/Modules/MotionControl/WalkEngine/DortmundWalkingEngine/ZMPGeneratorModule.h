/**
 * @file ZMPGeneratorModule.h
 *
 * Module wrapper for the ZMPGenerator
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
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/RefZMP.h"
#include "Representations/MotionControl/FootSteps.h"
#include "ZMPGenerator.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/ReferenceModificator.h"

MODULE(ZMPGeneratorModule)
REQUIRES(FootSteps)
// USES(WalkingInfo)
REQUIRES(ReferenceModificator)
REQUIRES(WalkingEngineParams)
REQUIRES(ControllerParams)
PROVIDES_WITH_MODIFY(RefZMP)
END_MODULE

class ZMPGeneratorModule : public ZMPGeneratorModuleBase {
public:
  ZMPGeneratorModule();

  ZMPGenerator generator;

  void update(RefZMP& theRefZMP);
};
