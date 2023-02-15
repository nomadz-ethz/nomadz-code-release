/**
 * @file IMUCoMProvider.cpp
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#include "IMUCoMProvider.h"
#include "Core/Math/Pose3D.h"
#include "Representations/Sensing/RobotModel.h"
#include "Core/Debugging/DebugDrawings.h"

void IMUCoMProvider::update(ActualCoM& theActualCoM) {

  DECLARE_PLOT("module:CoMProvider:ActualCoM.x");
  DECLARE_PLOT("module:CoMProvider:ActualCoM.y");

  static bool running = false;
  if (!theFootSteps.running && running) {
    // footPositions.clear();
    running = false;
    (Point&)theActualCoM = Point(0.0, 0.0);
    return;
  } else if (theFootSteps.running && !running) {
    running = true;
  }

  Point rcs = theWalkingInfo.toRobotCoords(theTargetCoM);

  rcs.rotateAroundX(theInertiaSensorData.angle.x);
  rcs.rotateAroundY(theInertiaSensorData.angle.y);

  (Point&)theActualCoM = theWalkingInfo.toWorldCoords(rcs);
  // Delete old foot step
  // footPositions.pop_front();

  PLOT("module:CoMProvider:ActualCoM.x", theActualCoM.x);
  PLOT("module:CoMProvider:ActualCoM.y", theActualCoM.y);
}

MAKE_MODULE(IMUCoMProvider, dortmundWalkingEngine)
