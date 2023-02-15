/**
 * @file PatternGeneratorModule.h
 *
 * Module wrapper for PatternGenerator
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Core/Module/Module.h"
#include "PatternGenerator.h"
#include "Representations/MotionControl/ReferenceModificator.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/Modeling/BallModel.h"

MODULE(PatternGeneratorModule)
REQUIRES(WalkingEngineParams)
REQUIRES(PatternGenRequest)
REQUIRES(RobotModel)
REQUIRES(RobotDimensions)
REQUIRES(FallDownState)
REQUIRES(ControllerParams)
REQUIRES(MotionSelection)
REQUIRES(FrameInfo)
REQUIRES(BallModel)
REQUIRES(BallModelAfterPreview)
USES(WalkingInfo)
USES(ReferenceModificator)
PROVIDES_WITH_MODIFY(FootSteps)
PROVIDES_WITH_MODIFY(SpeedInfo)
END_MODULE

class PatternGeneratorModule : public PatternGeneratorModuleBase {
public:
  PatternGeneratorModule();

private:
  PatternGenerator patternGen;
  void update(SpeedInfo& speedInfo);
  void update(FootSteps& steps);
};
