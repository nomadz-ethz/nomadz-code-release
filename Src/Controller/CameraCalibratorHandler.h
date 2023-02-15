/**
 * @file CameraCalibratorHandler.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#pragma once

#include "ImageViewAdapter.h"
#include "Core/MessageQueue/MessageQueue.h"

class RobotConsole;

class CameraCalibratorHandler : public PointListener {
private:
  RobotConsole* robotConsole;

public:
  CameraCalibratorHandler(RobotConsole* console) : robotConsole(console) {}

  void deliverPoint(const Vector2<int>& point);

  void setActive(std::string view);

  void setInactive(std::string view);
};
