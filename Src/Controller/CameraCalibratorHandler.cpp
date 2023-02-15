/**
 * @file CameraCalibratorHandler.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#include "CameraCalibratorHandler.h"
#include "Core/Debugging/DebugRequest.h"
#include "Controller/ImageViewAdapter.h"
#include "Controller/RobotConsole.h"
#include "Core/Debugging/Debugging.h"
#include <sstream>

void CameraCalibratorHandler::deliverPoint(const Vector2<int>& point) {
  std::stringstream line;
  line << "set module:CameraCalibrator:point x = ";
  line << point.x;
  line << "; y = ";
  line << point.y;
  line << ";";
  robotConsole->handleConsoleLine(line.str());
}
void CameraCalibratorHandler::setActive(std::string view) {
  if (ImageViewAdapter::addListener(this, view)) {
    OUTPUT(idText, text, "CameraCalibratorHandler activated for \"" + view + "\" ImageView");
  } else {
    OUTPUT(idText, text, "CameraCalibratorHandler is already active for \"" + view + "\" ImageView");
  }
}
void CameraCalibratorHandler::setInactive(std::string view) {
  ImageViewAdapter::removeListener(this, view);
  OUTPUT(idText, text, "CameraCalibratorHandler deactivated for \"" + view + "\"");
}
