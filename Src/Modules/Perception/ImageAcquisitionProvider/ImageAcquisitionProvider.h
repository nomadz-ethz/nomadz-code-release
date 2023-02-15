/**
 * @file ImageAcquisitionProvider.h
 *
 * This file declares a module which can save images and the camera matrix automatically.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Perception/ImageAcquisition.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Core/System/SystemCall.h"
#include "Core/System/Time.h"

MODULE(ImageAcquisitionProvider)
PROVIDES_WITH_MODIFY(ImageAcquisition)
REQUIRES(Image)
REQUIRES(RobotInfo)
REQUIRES(CameraMatrix)
REQUIRES(CameraInfo)
REQUIRES(BallPercept)
REQUIRES(BallSpots)
LOADS_PARAMETER(bool, activated)
LOADS_PARAMETER(unsigned int, acquisitionRateInMilliseconds)
LOADS_PARAMETER(bool, saveCameraMatrix)
LOADS_PARAMETER(bool, saveBallPatches)
LOADS_PARAMETER(unsigned int, colorSpace)
LOADS_PARAMETER(unsigned int, selectedCamera)
LOADS_PARAMETER(bool, obtainRFData)
END_MODULE

class ImageAcquisitionProvider : public ImageAcquisitionProviderBase {

private:
  // Last saving
  unsigned int lastSaving = Time::getRealSystemTime();
  bool initialized = false;

public:
  // Provides ImageAcquisition
  void update(ImageAcquisition& imageAcquisition);
  void saveDataToFolderRF(ImageAcquisition& imageAcquisition, const std::string& folderPath);
  void saveDataToFolder(ImageAcquisition& imageAcquisition, const std::string& folderPath);
};
