/**
 * @file ZMPGeneratorModule.cpp
 *
 * Module wrapper for the ZMPGenerator
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#include "ZMPGeneratorModule.h"
#include "Core/Debugging/Debugging.h"
#include "Core/Debugging/DebugDrawings.h"

ZMPGeneratorModule::ZMPGeneratorModule()
    : generator(theFootSteps,
                theWalkingEngineParams,
                theControllerParams,
                // theWalkingInfo,
                theReferenceModificator) {}

void ZMPGeneratorModule::update(RefZMP& theRefZMP) {
  generator.updateRefZMP(theRefZMP);
  PLOT("module:ZMPGenerator:unrotatedZMP.x", generator.plotZMP.x);
  PLOT("module:ZMPGenerator:unrotatedZMP.y", generator.plotZMP.y);
};

MAKE_MODULE(ZMPGeneratorModule, dortmundWalkingEngine)