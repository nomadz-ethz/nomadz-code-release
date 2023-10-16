/**
 * @file JoystickImageController.h
 *
 * Handles image data acquisition triggered by module JoystickImageController
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/MotionControl/JoystickControl.h"
#include "Representations/Perception/JoystickImageControl.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"

MODULE(JoystickImageController)

REQUIRES(Image)
REQUIRES(CameraMatrix)
REQUIRES(CameraInfo)
REQUIRES(JoystickControl)
PROVIDES_WITH_MODIFY(JoystickImageControl)

END_MODULE

class JoystickImageController : public JoystickImageControllerBase {

private:
public:
  JoystickImageController();  // Constructor
  ~JoystickImageController(); // Destructor

  // Update function
  void update(JoystickImageControl& joystickImageControl);

  // Helper functions
  void saveDataToFolder(const std::string& folderPath);
};
