/**
 * @file JoystickImageController.cpp
 *
 * This file implements the module JoystickImageController.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "JoystickImageController.h"

#include "Core/System/File.h"
#include "Core/System/Time.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/stat.h>
#include <sys/types.h>

#include <iostream>

MAKE_MODULE(JoystickImageController, Perception)

JoystickImageController::JoystickImageController() {}

JoystickImageController::~JoystickImageController() {}

void JoystickImageController::update(JoystickImageControl& joystickImageControl) {

  joystickImageControl.upperImageTaken = false;
  joystickImageControl.lowerImageTaken = false;

  if (theJoystickControl.acquireDataUpper && theCameraInfo.camera == theCameraInfo.upper) {
    saveDataToFolder(theJoystickControl.pathData);
    joystickImageControl.upperImageTaken = true;
    if (theJoystickControl.printOuts) {
      std::cout << "Acquire data upper successful!" << std::endl;
    }
  }

  if (theJoystickControl.acquireDataLower && theCameraInfo.camera == theCameraInfo.lower) {
    saveDataToFolder(theJoystickControl.pathData);
    joystickImageControl.lowerImageTaken = true;
    if (theJoystickControl.printOuts) {
      std::cout << "Acquire data lower successful!" << std::endl;
    }
  }
}

void JoystickImageController::saveDataToFolder(const std::string& folderPath) {
  // Create the folder
  struct stat s;
  const bool dirExists = (stat(folderPath.c_str(), &s) == 0 && S_ISDIR(s.st_mode));
  if (!dirExists) {
    std::cout << "Dir does not exist!" << std::endl;
    int status = mkdir(folderPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (status != 0) {
      std::cout << "Path could not be created..." << std::endl;
      // throw std::runtime_error(std::string("JoystickImageController::saveDataToFolder: could not mkdir ") + folderPath);
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
    std::cout << "path: " << path << std::endl;
    OutMapFile stream(path);
    stream << theCameraInfo;
  }

  {
    const std::string path = pathPrefix + "idCameraMatrix.txt";
    std::cout << "path: " << path << std::endl;
    OutMapFile stream(path);
    stream << theCameraMatrix;
  }
}
