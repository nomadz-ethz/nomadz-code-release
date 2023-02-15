/**
 * @file TiltControllerModule.cpp
 *
 * Module wrapper for the TiltController
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#include "TiltControllerModule.h"

TiltControllerModule::TiltControllerModule()
    : controller(theInertiaSensorData, theWalkingEngineParams /*,
		theJointCalibration,
		theJointRequest,
		theSpeedInfo,
		theTargetCoM*/) {}

void TiltControllerModule::update(BodyTilt& theBodyTilt) {
  controller.updateBodyTilt(theBodyTilt);
};

MAKE_MODULE(TiltControllerModule, dortmundWalkingEngine)