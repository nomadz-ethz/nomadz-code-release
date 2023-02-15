/**
 * @file ZMPIPControllerModule.cpp
 *
 * Module wrapper for the ZMP/IP-Controller
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#include "ZMPIPControllerModule.h"
#include "Core/Debugging/DebugDrawings.h"
ZMPIPControllerModule::ZMPIPControllerModule()
    : controller(theRobotModel,
                 theRefZMP,
                 theInertiaSensorData,
                 theWalkingEngineParams,
                 thePatternGenRequest,
                 theFallDownState,
                 theControllerParams,
                 theZMPModel,
                 theWalkingInfo) {}

void ZMPIPControllerModule::update(TargetCoM& theTargetCoM) {
  Vector3f x, y;
  controller.updateKinematicRequest(theTargetCoM);
  controller.getObservations(x, y);
  PLOT("module:ZMPIPController:ZMP.x", x[2]);
  PLOT("module:ZMPIPController:ZMP.y", y[2]);

  PLOT("module:ZMPIPController:Spd.x", x[1]);
  PLOT("module:ZMPIPController:Spd.y", y[1]);

  PLOT("module:ZMPIPController:Pos.x", x[0]);
  PLOT("module:ZMPIPController:Pos.y", y[0]);
};

MAKE_MODULE(ZMPIPControllerModule, dortmundWalkingEngine)