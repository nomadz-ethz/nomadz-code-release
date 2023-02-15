/**
 * @file SwingLegControllerModule.cpp
 *
 * Module wrapper for the SwingLegController
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#include "SwingLegControllerModule.h"
#include "Core/Debugging/DebugDrawings3D.h"

SwingLegControllerModule::SwingLegControllerModule()
    : controller(theWalkingEngineParams,
                 theFootSteps,
                 theBallModel,
                 theMotionRequest,
                 theWalkingInfo,
                 theFallDownState,
                 thePatternGenRequest,
                 theObservedError,
                 theControllerParams) {}

void SwingLegControllerModule::update(Footpositions& footpositions) {
  controller.updateFootpositions(footpositions);

  DECLARE_DEBUG_DRAWING3D("Preview", "robot");
  if (!controller.phases.empty()) {
    for (SwingLegController::PhaseList::iterator p = ++controller.phases.begin(); p != controller.phases.end(); p++) {
      for (int footNum = 0; footNum < 2; footNum++) {
        Point fp = p->members.front().footPos[footNum];
        Vector2<> fpRCS = theWalkingInfo.toRobotCoords(Vector2<>(fp.x, fp.y));
        SPHERE3D("Preview", fpRCS.x * 1000, fpRCS.y * 1000, -230, 20, ColorClasses::blue);
      }
    }
  }
};

void SwingLegControllerModule::update(ReferenceModificator& referenceModificator) {
  controller.update(referenceModificator);
};

MAKE_MODULE(SwingLegControllerModule, dortmundWalkingEngine)