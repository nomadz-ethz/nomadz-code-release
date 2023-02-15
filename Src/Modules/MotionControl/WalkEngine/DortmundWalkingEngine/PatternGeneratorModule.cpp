/**
 * @file PatternGeneratorModule.cpp
 *
 * Module wrapper for PatternGenerator
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#include "PatternGeneratorModule.h"

PatternGeneratorModule::PatternGeneratorModule()
    : patternGen(theWalkingEngineParams,
                 thePatternGenRequest,
                 theRobotModel,
                 theRobotDimensions,
                 theFallDownState,
                 theControllerParams,
                 theMotionSelection,
                 theWalkingInfo,
                 theReferenceModificator,
                 theFrameInfo,
                 theBallModel,
                 theBallModelAfterPreview) {}

void PatternGeneratorModule::update(FootSteps& steps) {
  patternGen.updateFootSteps(steps);
}

void PatternGeneratorModule::update(SpeedInfo& speedInfo) {
  patternGen.updateSpeedInfo(speedInfo);
}

MAKE_MODULE(PatternGeneratorModule, dortmundWalkingEngine)
