/**
 * @file SwingLegControllerModule.h
 *
 * Module wrapper for the SwingLegController
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Core/Module/Module.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/MotionControl/FootSteps.h"
#include "SwingLegController.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/ObservedError.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/ReferenceModificator.h"

MODULE(SwingLegControllerModule)
REQUIRES(WalkingEngineParams)
REQUIRES(ControllerParams)
REQUIRES(FootSteps)
REQUIRES(MotionRequest)
REQUIRES(ObservedError)
REQUIRES(BallModel)
REQUIRES(PatternGenRequest)
REQUIRES(FallDownState)
USES(WalkingInfo)
PROVIDES_WITH_MODIFY(Footpositions)
PROVIDES_WITH_MODIFY(ReferenceModificator)
END_MODULE

class SwingLegControllerModule : public SwingLegControllerModuleBase {
public:
  SwingLegControllerModule();

  SwingLegController controller;

  void update(Footpositions& footpositions);
  void update(ReferenceModificator& referenceModificator);
};
