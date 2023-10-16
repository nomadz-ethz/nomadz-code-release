/**
 * @file JoystickController.cpp
 *
 * This file implements the module JoystickController.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include "JoystickController.h"

#include "Core/System/File.h"
#include "Core/System/SystemCall.h"
#include "Core/System/Time.h"

#include "Core/Debugging/Debugging.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>

#include <iostream>

MAKE_MODULE(JoystickController, Motion Control)

JoystickController::JoystickController() {

  messageOK = false;
  initJoystickControl = false;

  joystickInitialized = false;
  motionInitialized = false;
  headAngleInitialized = false;

  playSound1.init(frequencyButtons);
  playSound2.init(frequencyButtons);

  headSpeedButton.init(frequencyButtons);
  headSpeed = headSpeedBound.at(1);

  sittingMode.init(frequencyButtons);
  standingMode.init(frequencyButtons);
  walkingMode.init(frequencyButtons);
  walkingSpeedButton.init(frequencyButtons);
  walkingSpeed = walkingSpeedBound.at(1);
  walkingSpeedBoundXDir.assign(3, 0.f);
  walkingSpeedBoundYDir.assign(3, 0.f);
  updateWalkingSpeedParameters();
  inWalk = false;

  kickButtonLeft.init(frequencyButtons);
  kickButtonRight.init(frequencyButtons);

  acquireDataUpperButton.init(frequencyData);
  acquireDataLowerButton.init(frequencyData);
  acquireDataUpper = false;
  acquireDataLower = false;
}

JoystickController::~JoystickController() {}

void JoystickController::updateJoystickControl() {
  if (useJoystickControl) {

    if (!joystickInitialized) {
      joystickInitialized = openSocket(socketIP.c_str());
      readCnt = 0;
      lastData[0] = 0;
    } else {
      getJoystickInput();
    }
  }
}

void JoystickController::update(HeadAngleRequest& headAngleRequest) {

  if (!joystickInitialized) {
    return;
  }

  // Initialize head motion request parameters
  if (!headAngleInitialized) {
    headAngleRequest.pan = headPanBound.at(1);
    headAngleRequest.tilt = headTiltBound.at(1);
    headAngleRequest.speed = headSpeedBound.at(1);

    headAngleInitialized = true;
  }

  if (!messageOK) {
    return;
  }
  // Update head angle pan
  headAngleRequest.pan = XBoxButton::getAxisCommand((float)axisCommands.at(RWH_AXIS), headPanBound, true);

  // Update head angle tilt
  headAngleRequest.tilt = XBoxButton::getAxisCommand((float)axisCommands.at(RWV_AXIS), headTiltBound, true);

  // Update head speed
  float headSpeedCmd = (float)axisCommands.at(CV_AXIS);
  if (headSpeedButton.update((headSpeedCmd != 0), frequencyButtons)) {
    float newHeadSpeed = headSpeed + headSpeedCmd * headSpeedStep * (-1.f); // times -1 to mirror axis
    if (newHeadSpeed < headSpeedBound.at(2) && newHeadSpeed >= headSpeedBound.at(0)) {
      if (printOuts) {
        std::stringstream ss;
        ss << "Change head speed to " << newHeadSpeed << "!" << std::endl;
        OUTPUT_TEXT(ss.str());
      }
      headSpeed = newHeadSpeed;
      headAngleRequest.speed = headSpeed;
    }
  }
}

void JoystickController::update(MotionRequest& motionRequest) {

  if (!joystickInitialized) {
    return;
  }

  // Initialize motion request parameters
  if (!motionInitialized) {
    motionRequest.motion = MotionRequest::Motion::specialAction;
    motionRequest.specialActionRequest.specialAction = SpecialActionRequest::stand;
    inWalk = false;
    motionRequest.walkRequest.mode = WalkRequest::speedMode;
    motionInitialized = true;
  }

  if (!messageOK) {
    return;
  }

  // Change mode to "sit"
  if (sittingMode.update((bool)buttonCommands.at(X_BUTTON), frequencyButtons)) {
    if (printOuts) {
      OUTPUT_TEXT("Sit!");
    }
    motionRequest.motion = MotionRequest::Motion::specialAction;
    motionRequest.specialActionRequest.specialAction = SpecialActionRequest::sitDown;
    inWalk = false;
  }

  // Change mode to "stand"
  if (standingMode.update((bool)buttonCommands.at(Y_BUTTON), frequencyButtons)) {
    if (printOuts) {
      OUTPUT_TEXT("Stand!");
    }
    motionRequest.motion = MotionRequest::Motion::specialAction;
    motionRequest.specialActionRequest.specialAction = SpecialActionRequest::stand;
    inWalk = false;
  }

  // Change mode to "walk"
  if (walkingMode.update((bool)buttonCommands.at(LW_BUTTON), frequencyButtons)) {
    if (printOuts) {
      OUTPUT_TEXT("Walk!");
    }
    motionRequest.motion = MotionRequest::Motion::walk;
    inWalk = true;
  }

  // Update walking speed parameter
  float walkingSpeedCmd = (float)axisCommands.at(CH_AXIS);
  if (walkingSpeedButton.update((walkingSpeedCmd != 0), frequencyButtons)) {
    float newWalkingSpeed = walkingSpeed + walkingSpeedCmd * walkingSpeedStep;
    if (newWalkingSpeed <= walkingSpeedBound.at(2) && newWalkingSpeed >= walkingSpeedBound.at(0)) {
      if (printOuts) {
        std::stringstream ss;
        ss << "Change head speed to " << newWalkingSpeed << "!";
        OUTPUT_TEXT(ss.str());
      }

      walkingSpeed = newWalkingSpeed;
      updateWalkingSpeedParameters();
    }
  }

  if (inWalk) {
    float xWalkCmd = XBoxButton::getAxisCommand((float)axisCommands.at(LWV_AXIS), walkingSpeedBoundXDir, true);
    motionRequest.walkRequest.speed.translation.x = xWalkCmd;

    float yWalkCmd = XBoxButton::getAxisCommand((float)axisCommands.at(LWH_AXIS), walkingSpeedBoundYDir, true);
    motionRequest.walkRequest.speed.translation.y = yWalkCmd;

    float posRotCmd = walkingRotScale * PI_2 * (float)axisCommands.at(LT_AXIS);
    float negRotCmd = walkingRotScale * PI_2 * (float)axisCommands.at(RT_AXIS) * (-1.f);
    float rotWalkCmd = posRotCmd + negRotCmd;
    motionRequest.walkRequest.speed.rotation = rotWalkCmd;
  }

  // Kicking
  // -> Right leg
  if (kickButtonRight.update((bool)buttonCommands.at(B_BUTTON), frequencyButtons)) {
    if (printOuts) {
      OUTPUT_TEXT("Kick with right foot!");
    }
    motionRequest.kickRequest.kickMotionType = KickRequest::fastKick;
    motionRequest.kickRequest.mirror = false;
    motionRequest.motion = MotionRequest::Motion::kick;
    inWalk = false;
  }

  // -> Left leg
  if (kickButtonLeft.update((bool)buttonCommands.at(A_BUTTON), frequencyButtons)) {
    if (printOuts) {
      OUTPUT_TEXT("Kick with left foot!");
    }
    motionRequest.kickRequest.kickMotionType = KickRequest::fastKick;
    motionRequest.kickRequest.mirror = false;
    OUTPUT_WARNING("Left kick currently disabled! Kicking right instead!");
    motionRequest.motion = MotionRequest::Motion::kick;
    inWalk = false;
  }
}

void JoystickController::update(JoystickControl& joystickControl) {

  if (!initJoystickControl) {
    initJoystickControl = true;

    joystickControl.pathData = pathData;
    joystickControl.printOuts = printOuts;
    joystickControl.acquireDataUpper = false;
    joystickControl.acquireDataLower = false;
  }

  updateJoystickControl();

  if (joystickInitialized) {

    if (messageOK) {

      // Play sound
      if (playSound1.update((bool)buttonCommands.at(START_BUTTON), frequencyButtons)) {
        if (printOuts) {
          OUTPUT_TEXT("Play sound 1!");
        }
        SystemCall::playSound("weeeee.wav");
      }
      if (playSound2.update((bool)buttonCommands.at(BACK_BUTTON), frequencyButtons)) {
        if (printOuts) {
          OUTPUT_TEXT("Play sound 2!");
        }
        SystemCall::playSound("Nao.wav");
      }

      // Acquire data (image + cameraInfo + cameraMatrix)
      if (acquireDataUpperButton.update((bool)buttonCommands.at(RB_BUTTON), frequencyData)) {
        acquireDataUpper = true;
        if (printOuts) {
          OUTPUT_TEXT("Take image upper!");
        }
      }

      if (acquireDataUpper && theCameraInfo.camera == theCameraInfo.upper) {
        if (saveDataToFolder(pathData)) {
          OUTPUT_TEXT("Upper camera image saved!");
        }
        acquireDataUpper = false;
      }

      // -> Lower
      if (acquireDataLowerButton.update((bool)buttonCommands.at(LB_BUTTON), frequencyData)) {
        acquireDataLower = true;
        if (printOuts) {
          OUTPUT_TEXT("Take image lower!");
        }
      }

      if (acquireDataLower && theCameraInfo.camera == theCameraInfo.lower) {
        if (saveDataToFolder(pathData)) {
          OUTPUT_TEXT("Lower camera image saved!");
        }
        acquireDataLower = false;
      }
    }
  }
}

void JoystickController::getJoystickInput() {
  // Read message from server
  char msg[1024] = {0};
  bool valid = false;
  int cnt = recv(joystickSocket, msg, 1024, 0);
  // Skip if read failed
  if (cnt < 0) {
    return;
  }
  // Read actual message
  for (int i = 0; i < cnt; ++i) {
    if (msg[i] == '\n') {
      std::memcpy(lastData, readData, sizeof(char) * readCnt);
      lastData[readCnt + i] = 0;
      valid = true;
      readCnt = 0;
      continue;
    }
    readData[readCnt] = msg[i];
    ++readCnt;
    assert(readCnt != 1024);
  }
  // Update if new line is read
  if (valid) {
    parseMessage(lastData);
  }
}

void JoystickController::updateWalkingSpeedParameters() {

  walkingSpeedBoundXDir.at(0) = -walkingSpeed * (walkingSpeedPercBackward / 100.f);
  walkingSpeedBoundXDir.at(2) = walkingSpeed;

  walkingSpeedBoundYDir.at(0) = -walkingSpeed * (walkingSpeedPercYDir / 100.f);
  walkingSpeedBoundYDir.at(2) = walkingSpeed * (walkingSpeedPercYDir / 100.f);

  walkingRotScale = (walkingSpeed / 10.f) * (walkingSpeedPercRot / 100.f);
}

bool JoystickController::saveDataToFolder(const std::string& folderPath) {
  // Create the folder
  struct stat s;
  const bool dirExists = (stat(folderPath.c_str(), &s) == 0 && S_ISDIR(s.st_mode));
  if (!dirExists) {
    OUTPUT_ERROR("Data dir does not exist! Creating...");
    int status = mkdir(folderPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (status != 0) {
      OUTPUT_ERROR("Path could not be created - image not saved!");
      return false;
    } else {
      OUTPUT_TEXT("Data dir was created successfully!");
    }
  }

  unsigned int systemTime = Time::getRealSystemTime();
  const std::string pathPrefix = folderPath + "/" + std::to_string(systemTime) + ".";

  // Export Images as .jpegs
  {
    const std::string path = pathPrefix + "idImage.bmp";
    cv::Mat mat = theImage.convertToCVMat();
    cv::imwrite(path, mat);
  }

  {
    const std::string path = pathPrefix + "idCameraInfo.txt";
    OutMapFile stream(path);
    stream << theCameraInfo;
  }

  {
    const std::string path = pathPrefix + "idCameraMatrix.txt";
    OutMapFile stream(path);
    stream << theCameraMatrix;
  }

  return true;
}

bool JoystickController::openSocket(const char* IP) {
  struct sockaddr_in serv_addr;

  if ((joystickSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    OUTPUT_ERROR("\n Socket creation error \n");
    return false;
  }

  memset(&serv_addr, '0', sizeof(serv_addr));

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORT);

  // Convert IPv4 and IPv6 addresses from text to binary form
  if (inet_pton(AF_INET, IP, &serv_addr.sin_addr) <= 0) {
    OUTPUT_ERROR("\nInvalid address/ Address not supported \n");
    close(joystickSocket);
    return false;
  }

  if (connect(joystickSocket, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    OUTPUT_ERROR("\nConnection Failed \n");
    close(joystickSocket);
    return false;
  }

  // Set non-blocking
  if (fcntl(joystickSocket, F_SETFL, fcntl(joystickSocket, F_GETFL) | O_NONBLOCK) < 0) {
    OUTPUT_ERROR("\nNon-blocking Failed \n");
    close(joystickSocket);
    return false;
  }

  return true;
}

void JoystickController::parseMessage(std::string msg) {
  char* cstr = new char[msg.length() + 1];
  strcpy(cstr, msg.c_str());
  char* p = strtok(cstr, " ");

  buttonCommands.clear();
  axisCommands.clear();

  int iter = 0;
  while (p != 0) {
    if (iter < numOfButtons) {
      buttonCommands.push_back(double(atof(p)));
    } else {
      axisCommands.push_back(double(atof(p)));
    }
    p = strtok(NULL, " ");
    iter++;
  }
  delete[] cstr;

  if (iter != (numOfAxes + numOfButtons)) {
    OUTPUT_ERROR("Joystick message corrupted!");
    messageOK = false;
  }

  messageOK = true;
}
