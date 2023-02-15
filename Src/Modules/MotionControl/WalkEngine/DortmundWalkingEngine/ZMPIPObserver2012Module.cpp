/**
 * @file ZMPIPObserver2012Module.cpp
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#include "ZMPIPObserver2012Module.h"
#include "Core/Debugging/DebugDrawings.h"
ZMPIPObserver2012Module::ZMPIPObserver2012Module()
    : observer(theControllerParams,
               theTargetCoM,
               thePatternGenRequest,
               theZMPModel,
               theWalkingInfo,
               theWalkingEngineParams,
               theActualCoM) {}

void ZMPIPObserver2012Module::update(ObservedError& theObservedError) {
  observer.update(theObservedError);
};

MAKE_MODULE(ZMPIPObserver2012Module, dortmundWalkingEngine)