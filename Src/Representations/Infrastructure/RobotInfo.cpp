/**
 * @file RobotInfo.cpp
 *
 * The file declares a class that encapsulates the structure RobotInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 * It also maps the robot's name on the robot's model.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * and <a href="mailto:aschreck@informatik.uni-bremen.de">André Schreck</a>
 */

#include "RobotInfo.h"
#include <cstring>
#include "Core/System/BHAssert.h"

bool RobotInfo::hasFeature(const RobotFeature feature) const {
  switch (feature) {
  case hands:
  case wristYaws:
  case tactileHandSensores:
    return naoBodyType >= H25;
  case tactileHeadSensores:
  case headLEDs:
    return naoHeadType >= H25;
  case grippyFingers:
    return naoBodyType >= H25 && naoVersion >= V5;
  case zGyro:
    return naoVersion >= V5;
  default:
    ASSERT(false);
    return false;
  }
}
