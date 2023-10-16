/**
 * @file JoystickController.h
 *
 * This file declares a module which updates the input coming from a joystick
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/JoystickControl.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/JoystickImageControl.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"

#include "Core/Math/Vector3.h"

#include "XBoxJoystick.h"
#include "XBoxButton.h"

#include <vector>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

MODULE(JoystickController)

REQUIRES(Image)
REQUIRES(CameraMatrix)
REQUIRES(CameraInfo)
PROVIDES_WITH_MODIFY(HeadAngleRequest)
PROVIDES_WITH_MODIFY(MotionRequest)
PROVIDES_WITH_MODIFY(JoystickControl)
LOADS_PARAMETER(std::vector<float>, headPanBound)
LOADS_PARAMETER(std::vector<float>, headTiltBound)
LOADS_PARAMETER(std::vector<float>, headSpeedBound)
LOADS_PARAMETER(float, headSpeedStep)
LOADS_PARAMETER(std::vector<float>, walkingSpeedBound)
LOADS_PARAMETER(float, walkingSpeedPercBackward)
LOADS_PARAMETER(float, walkingSpeedPercYDir)
LOADS_PARAMETER(float, walkingSpeedPercRot)
LOADS_PARAMETER(float, walkingSpeedStep)
LOADS_PARAMETER(unsigned int, frequencyButtons)
LOADS_PARAMETER(unsigned int, frequencyData)
LOADS_PARAMETER(std::string, pathData)
LOADS_PARAMETER(bool, printOuts)
LOADS_PARAMETER(std::string, socketIP)
LOADS_PARAMETER(bool, useJoystickControl)

END_MODULE

class JoystickController : public JoystickControllerBase {

private:
  const unsigned int PORT = 8080;

  int joystickSocket;
  std::vector<double> buttonCommands;
  std::vector<double> axisCommands;
  bool messageOK;

  bool joystickInitialized;
  bool motionInitialized;
  bool headAngleInitialized;

  bool initJoystickControl;

  XBoxJoystick xboxJoystick;

  XBoxButton playSound1;
  XBoxButton playSound2;

  XBoxButton headSpeedButton;
  float headSpeed;

  XBoxButton sittingMode;
  XBoxButton standingMode;
  XBoxButton walkingMode;
  XBoxButton walkingSpeedButton;
  float walkingSpeed;
  std::vector<float> walkingSpeedBoundXDir;
  std::vector<float> walkingSpeedBoundYDir;
  float walkingRotScale;
  bool inWalk;

  XBoxButton kickButtonRight;
  XBoxButton kickButtonLeft;

  XBoxButton acquireDataUpperButton;
  XBoxButton acquireDataLowerButton;
  bool acquireDataUpper;
  bool acquireDataLower;

  const float PI_2 = 1.570796f;

  int readCnt = 0;
  char readData[1024];
  char lastData[1024];

public:
  JoystickController();  // Constructor
  ~JoystickController(); // Destructor

  // Update functions
  void update(HeadAngleRequest& headAngleRequest);
  void update(MotionRequest& motionRequest);
  void update(JoystickControl& joystickControl);

  // Helper functions
  void updateJoystickControl();
  void getJoystickInput();
  void updateWalkingSpeedParameters();
  bool saveDataToFolder(const std::string& folderPath);
  bool openSocket(const char* IP);
  void parseMessage(std::string msg);
};
