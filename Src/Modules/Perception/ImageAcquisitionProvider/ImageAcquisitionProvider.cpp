/**
 * @file ImageAcquisitionProvider.cpp
 *
 * This File implements the  module ImageAcquisitionProvider.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "ImageAcquisitionProvider.h"
#include "Core/System/File.h"
#include "Core/System/Time.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/stat.h>
#include <sys/types.h>

#include <iostream>

using namespace cv;
using namespace std;

MAKE_MODULE(ImageAcquisitionProvider, Perception)

void ImageAcquisitionProvider::update(ImageAcquisition& imageAcquisition) {

  // TODO Rename module to ImageAcquisition? or ImageAcquirer? and representation to ImageAcquisitionOutput?
  // TODO Just use something like parameters:ImageAcquisition instead;
  //      representation:ImageAcquisition should only contain output information, not parameters
  //      (maybe like how many pictures taken? etc)
  if (!initialized) {
    imageAcquisition.activated = activated;

    switch (selectedCamera) {

    case 1:
      imageAcquisition.selectedCamera = imageAcquisition.lower;
      break;
    case 2:
      imageAcquisition.selectedCamera = imageAcquisition.both;
      break;
    case 3:
      imageAcquisition.selectedCamera = imageAcquisition.JerseyNRBased;
      break;
    case 0:
    default:
      imageAcquisition.selectedCamera = imageAcquisition.upper;
      break;
    }
    switch (colorSpace) {

    case 1:
      imageAcquisition.colorSpace = imageAcquisition.RGB;
      break;
    case 0:
    default:
      imageAcquisition.colorSpace = imageAcquisition.YCrCb;
      break;
    }

    imageAcquisition.acquisitionRateInMilliseconds = acquisitionRateInMilliseconds;
    imageAcquisition.saveCameraMatrix = saveCameraMatrix;
    imageAcquisition.saveBallPatches = saveBallPatches;
    imageAcquisition.obtainRFData = obtainRFData;
    initialized = true;
  }

  // Get system time
  unsigned int systemTime = Time::getRealSystemTime();

  // Check which camera has been selected
  int selected;
  // If selectedCamera = 2, the odd robots acquire with upper camera, even robots acquire images with lower camera
  switch (imageAcquisition.selectedCamera) {
  case ImageAcquisition::upper:
    selected = theCameraInfo.upper;
    break;
  case ImageAcquisition::lower:
    selected = theCameraInfo.lower;
    break;
  case ImageAcquisition::both:
    selected = theCameraInfo.camera;
    break;
  case ImageAcquisition::JerseyNRBased:
    selected = (theRobotInfo.number % 2 == 1) ? theCameraInfo.upper : theCameraInfo.lower;
    break;
  }

  bool enoughTimePassed = systemTime >= (lastSaving + imageAcquisition.acquisitionRateInMilliseconds);

  // Check whether camera matrix is valid if needed
  bool cameraMatrixValid = !(imageAcquisition.saveCameraMatrix) || theCameraMatrix.isValid;

  // Check whether image acquisition is activated, enough time has passed, image is from upper camera and camera matrix is
  // valid
  if (imageAcquisition.activated && enoughTimePassed && theCameraInfo.camera == selected && cameraMatrixValid) {

    // Determines path of images
    string pathImages = string(File::getBHDir()) + "/Images/";
    if (imageAcquisition.obtainRFData) {
      saveDataToFolderRF(imageAcquisition, pathImages);
    } else {
      saveDataToFolder(imageAcquisition, pathImages);
    }
  }
}

void ImageAcquisitionProvider::saveDataToFolderRF(ImageAcquisition& imageAcquisition, const std::string& folderPath) {
  // Create the folder
  struct stat s;
  const bool dirExists = (stat(folderPath.c_str(), &s) == 0 && S_ISDIR(s.st_mode));
  if (!dirExists) {
    std::cout << "Dir does not exist!" << std::endl;
    int status = mkdir(folderPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (status != 0) {
      std::cout << "Path could not be created..." << std::endl;
    }
  }

  unsigned int systemTime = Time::getRealSystemTime();
  lastSaving = systemTime;
  const std::string pathPrefix = folderPath + "/" + std::to_string(systemTime) + ".";

  // Export Images as .jpegs
  {
    const std::string path = pathPrefix + "idImage.bmp";
    cv::Mat mat = theImage.convertToCVMat();
    // Convert to BGR mat (openCV needs BGR mat to save as RGB image...*ç%/*ç%)
    if (imageAcquisition.colorSpace == imageAcquisition.RGB) {

      // Convert from YCrCb to BGR --> imwrite will save to RGB
      cvtColor(mat, mat, COLOR_YCrCb2BGR);
    }
    if (imageAcquisition.colorSpace == imageAcquisition.YCrCb) {

      // Convert from YCrCb to CbCrY..imwrite will save to YCrCb
      cvtColor(mat, mat, COLOR_RGB2BGR);
    }

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

void ImageAcquisitionProvider::saveDataToFolder(ImageAcquisition& imageAcquisition, const std::string& pathImages) {
  unsigned int systemTime = Time::getRealSystemTime();
  // Save current time
  lastSaving = systemTime;

  // Make Image directory if necessary
  mkdir(pathImages.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  // Determine path of image
  string pathImage = pathImages + to_string(lastSaving) + ".bmp";

  // Creates new Image for cast
  Mat imageMat;
  imageMat = theImage.convertToCVMat();

  // Convert to BGR mat (openCV needs BGR mat to save as RGB image...*ç%/*ç%)
  if (imageAcquisition.colorSpace == imageAcquisition.RGB) {

    // Convert from YCrCb to BGR --> imwrite will save to RGB
    cvtColor(imageMat, imageMat, COLOR_YCrCb2BGR);
  }
  if (imageAcquisition.colorSpace == imageAcquisition.YCrCb) {

    // Convert from YCrCb to CbCrY..imwrite will save to YCrCb
    cvtColor(imageMat, imageMat, COLOR_RGB2BGR);
  }

  // Write image
  bool success = imwrite(pathImage, imageMat);

  if (!success) {
    OUTPUT(idText, text, "Could not save image; check if ~/Images on robot exists." << endl);
  }

  // Save Camera Matrix if needed

  if (imageAcquisition.saveCameraMatrix) {

    // Create xml directory if necessary
    string pathXML = pathImages + "xml/";
    mkdir(pathXML.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    FileStorage fs(pathXML + to_string(lastSaving) + ".xml", FileStorage::WRITE);

    if (fs.isOpened()) {

      // Convert CameraMatrix to opencv readable matrix
      const RotationMatrix* rot = &(theCameraMatrix.rotation);
      const Vector3<>* trans = &(theCameraMatrix.translation);
      Mat rotMatrix = (Mat_<double>(3, 3) << rot->c0.x,
                       rot->c0.y,
                       rot->c0.z,
                       rot->c1.x,
                       rot->c1.y,
                       rot->c1.z,
                       rot->c2.x,
                       rot->c2.y,
                       rot->c2.z);
      Mat transMatrix = (Mat_<double>(3, 1) << trans->x, trans->y, trans->z);
      fs << "CameraMatrix"
         << "{";
      fs << "Rotation" << rotMatrix;
      fs << "Translation" << transMatrix;
      fs << "}";
      fs.release();

      rotMatrix.release();
      transMatrix.release();
    } else {
      OUTPUT(idText, text, "Could not save camera matrix; check if ~/Images/xml on robot exists." << endl);
    }
  }

  // Save ball patches
  if (imageAcquisition.saveBallPatches) {

    const float marginFactor = 0.77245f; // Have margins that are 77.245% the radius (sqrt(pi) - 1)

    const int w = theCameraInfo.width;
    const int h = theCameraInfo.height;

    const std::string dirPatches = pathImages + "ballPatches/";
    mkdir(dirPatches.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    for (size_t i = 0, n = theBallSpots.ballSpots.size(); i < n; ++i) {

      const auto& spot = theBallSpots.ballSpots[i];
      const int r = (int)std::round(spot.radius * (1.0f + marginFactor));
      const int x = spot.position.x;
      const int y = spot.position.y;

      Vector2<> pointOnField;
      Geometry::calculatePointOnField(x, y, theCameraMatrix, theCameraInfo, pointOnField);
      const int distance = (int)std::round(pointOnField.abs());

      const int x0 = std::max(0, x - r);
      const int y0 = std::max(0, y - r);
      const int x1 = std::min(w - 1, x + r);
      const int y1 = std::min(h - 1, y + r);

      // fileName: i-distance-timestamp-{U/L}-cutLeft-cutRight-cutTop-cutBottom.bmp
      const std::string fileName = to_string(i) + "-" + to_string(distance) + "-" + to_string(lastSaving) + "-" +
                                   (theCameraInfo.camera == CameraInfo::upper ? "U" : "L") + "-" + to_string(x0 - (x - r)) +
                                   "-" + to_string((x + r) - x1) + "-" + to_string(y0 - (y - r)) + "-" +
                                   to_string((y + r) - y1) + ".bmp";

      cv::Mat patch(x1 - x0, y1 - y0, CV_8UC3);
      theImage.extractCVMatPatch(x0, y0, patch);
      cv::imwrite(dirPatches + fileName, patch);
    }
  }
}
