/**
 * @file CameraProvider.cpp
 *
 * This file declares a module that provides camera images.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#include <cstdio>
#include <unistd.h>

#include "CameraProvider.h"
#include "Representations/Perception/JPEGImage.h"
#include "Core/System/SystemCall.h"
#include "Core/System/Time.h"
#include "Core/Streams/InStreams.h"
#include "Core/Debugging/Stopwatch.h"

PROCESS_WIDE_STORAGE(CameraProvider) CameraProvider::theInstance = 0;

CameraProvider::CameraProvider()
    : currentImageCamera(0)
#ifdef CAMERA_INCLUDED
      ,
      imageTimeStamp(0), otherImageTimeStamp(0), lastImageTimeStamp(0), lastImageTimeStampLL(0), firstImageTimeStamp(0)
#endif
{
  InMapFile upperStream("upperCameraInfo.cfg");
  ASSERT(upperStream.exists());
  upperStream >> upperCameraInfo;
  InMapFile lowerStream("lowerCameraInfo.cfg");
  ASSERT(lowerStream.exists());
  lowerStream >> lowerCameraInfo;

#ifdef CAMERA_INCLUDED
#ifdef ROBOT_V6
  upperCamera = new NaoCamera("/dev/video-top", upperCameraInfo.camera, upperCameraInfo.width, upperCameraInfo.height, true);
  lowerCamera =
    new NaoCamera("/dev/video-bottom", lowerCameraInfo.camera, lowerCameraInfo.width, lowerCameraInfo.height, false);
#else // ROBOT_V5
  upperCamera = new NaoCamera("/dev/video0", upperCameraInfo.camera, upperCameraInfo.width, upperCameraInfo.height, true);
  lowerCamera = new NaoCamera("/dev/video1", lowerCameraInfo.camera, lowerCameraInfo.width, lowerCameraInfo.height, false);
#endif
  cycleTime = upperCamera->getFrameRate();
  ASSERT(upperCamera->getFrameRate() == lowerCamera->getFrameRate());
#else
  upperCamera = lowerCamera = NULL;
  cycleTime = 1.f / 30.f;
#endif
  theInstance = this;
}

CameraProvider::~CameraProvider() {
#ifdef CAMERA_INCLUDED
  if (upperCamera)
    delete upperCamera;
  if (lowerCamera)
    delete lowerCamera;
#endif
  theInstance = 0;
}

void CameraProvider::update(Image& image) {
#ifdef CAMERA_INCLUDED
  ASSERT(!currentImageCamera);

  // Apply a copy of the CameraSettings
  CameraSettings actualSettings;
  actualSettings.getChangesAndAssign(theCameraSettings);

  if (upperCamera->hasImage() && (!lowerCamera->hasImage() || upperCamera->getTimeStamp() < lowerCamera->getTimeStamp())) {
    image.setResolution(upperCameraInfo.width, upperCameraInfo.height);
    image.setImage(const_cast<unsigned char*>(upperCamera->getImage()));
    lastImageTimeStampLL = upperCamera->getTimeStamp();
    imageTimeStamp = image.timeStamp =
      std::max(lastImageTimeStamp + 1, (unsigned)(upperCamera->getTimeStamp() / 1000 - Time::getSystemTimeBase()));

    // Don't set autoWhiteBalance = 1 until some time after first image received
    if (actualSettings.autoWhiteBalance.value == 1 && imageTimeStamp - firstImageTimeStamp < 10000)
      actualSettings.autoWhiteBalance.value = 0;

    upperCamera->setSettings(actualSettings);
    upperCamera->writeCameraSettings();
    currentImageCamera = upperCamera;
  } else if (lowerCamera->hasImage()) {
    image.setResolution(lowerCameraInfo.width, lowerCameraInfo.height);
    image.setImage(const_cast<unsigned char*>(lowerCamera->getImage()));
    lastImageTimeStampLL = lowerCamera->getTimeStamp();
    otherImageTimeStamp = image.timeStamp =
      std::max(lastImageTimeStamp + 1, (unsigned)(lowerCamera->getTimeStamp() / 1000) - Time::getSystemTimeBase());

    // Don't set autoWhiteBalance = 1 until some time after first image received
    if (actualSettings.autoWhiteBalance.value == 1 && otherImageTimeStamp - firstImageTimeStamp < 10000)
      actualSettings.autoWhiteBalance.value = 0;

    lowerCamera->setSettings(actualSettings);
    lowerCamera->writeCameraSettings();
    currentImageCamera = lowerCamera;
  }
  ASSERT(image.timeStamp >= lastImageTimeStamp);
  if (firstImageTimeStamp == 0)
    firstImageTimeStamp = image.timeStamp;
  lastImageTimeStamp = image.timeStamp;
#endif // CAMERA_INCLUDED

  // Allow freezing camera image
  // !!Camera matrix representation tends to get messed up!!
  static int frozen = 0;
  static Image frozenUpper(false, upperCameraInfo.width, upperCameraInfo.height);
  static Image frozenLower(false, lowerCameraInfo.width, lowerCameraInfo.height);
  DEBUG_RESPONSE_ONCE("module:CameraProvider:freeze", { frozen = 1; });
  DEBUG_RESPONSE_ONCE("module:CameraProvider:unfreeze", { frozen = 0; });

  switch (frozen) {
  case 1: // Capture upper camera
    if (currentImageCamera == upperCamera) {
      frozenUpper = image;
      ++frozen;
    }
    break;
  case 2: // Capture lower camera
    if (currentImageCamera == lowerCamera) {
      frozenLower = image;
      ++frozen;
    }
    break;
  case 3: // Show a frozen image
    if (currentImageCamera == upperCamera) {
      image = frozenUpper;
    } else if (currentImageCamera == lowerCamera) {
      image = frozenLower;
    }
    break;
  }

  STOP_TIME_ON_REQUEST("compressJPEG",
                       { DEBUG_RESPONSE("representation:JPEGImage", OUTPUT(idJPEGImage, bin, JPEGImage(image));); });
}

void CameraProvider::update(FrameInfo& frameInfo) {
  frameInfo.time = theImage.timeStamp;
  frameInfo.cycleTime = cycleTime * 0.5f;
}

void CameraProvider::update(CognitionFrameInfo& cognitionFrameInfo) {
  cognitionFrameInfo.time = theImage.timeStamp;
  cognitionFrameInfo.cycleTime = cycleTime * 0.5f;
}

void CameraProvider::update(CameraInfo& cameraInfo) {
  if (currentImageCamera == upperCamera) {
    cameraInfo = upperCameraInfo;
  } else {
    cameraInfo = lowerCameraInfo;
  }
}

bool CameraProvider::isFrameDataComplete() {
#ifdef CAMERA_INCLUDED
  if (theInstance)
    return theInstance->upperCamera->hasImage() || theInstance->lowerCamera->hasImage();
  else
#endif
    return true;
}

void CameraProvider::waitForFrameData2() {
#ifdef CAMERA_INCLUDED

  const unsigned int timeout = 2000 * 10;

  if (currentImageCamera) {
    currentImageCamera->releaseImage();
    currentImageCamera = 0;
  }

  for (;;) {
    if (upperCamera->hasImage() || lowerCamera->hasImage())
      return;

    bool resetUpper = false;
    bool resetLower = false;
    if (!NaoCamera::captureNew(*upperCamera, *lowerCamera, timeout, resetUpper, resetLower)) {
      OUTPUT_WARNING("CameraProvider: Poll failed. Resetting both cameras");
      resetUpper = resetLower = true;
    } else {
      if (resetUpper)
        OUTPUT_WARNING("CameraProvider: Capturing image failed. Resetting upper camera.");
      if (resetLower)
        OUTPUT_WARNING("CameraProvider: Capturing image failed. Resetting lower camera.");
    }

    // FIXME: temporarily disable time checks while investigating problems with timing base
    unsigned int now = Time::getRealSystemTime();

    if (!resetUpper && imageTimeStamp && now - imageTimeStamp >= timeout) {
      OUTPUT_WARNING("CameraProvider: Capturing image timed out. Resetting upper camera.");
      resetUpper = true;
    }
    if (!resetLower && otherImageTimeStamp && now - otherImageTimeStamp >= timeout) {
      OUTPUT_WARNING("CameraProvider: Capturing image timed out. Resetting lower camera.");
      resetLower = true;
    }

#ifdef ROBOT_V6
    // always apply a hard reset on the v6 as otherwise apparent hang
    if (resetUpper || resetLower) {
      OUTPUT_WARNING("Resetting cameras...");
      system("/usr/libexec/reset-cameras.sh toggle");
      // FIXME: we seem to need to sleep around two seconds here for the script to fix, implement this more properly
      sleep(2);
      resetUpper = true;
      resetLower = true;
    }
#endif

    if (resetUpper || resetLower) {
      SystemCall::playSound("cameraReset.wav");
    }

    if (resetUpper) {
      delete upperCamera;
#ifdef ROBOT_V6
      upperCamera =
        new NaoCamera("/dev/video-top", upperCameraInfo.camera, upperCameraInfo.width, upperCameraInfo.height, true);
#else // ROBOT_V5
      upperCamera =
        new NaoCamera("/dev/video0", upperCameraInfo.camera, upperCameraInfo.width, upperCameraInfo.height, true);
#endif
      imageTimeStamp = 0;
      firstImageTimeStamp = 0;
    }

    if (resetLower) {
      delete lowerCamera;
#ifdef ROBOT_V6
      lowerCamera =
        new NaoCamera("/dev/video-bottom", lowerCameraInfo.camera, lowerCameraInfo.width, lowerCameraInfo.height, false);
#else
      lowerCamera =
        new NaoCamera("/dev/video1", lowerCameraInfo.camera, lowerCameraInfo.width, lowerCameraInfo.height, false);
#endif
      otherImageTimeStamp = 0;
      firstImageTimeStamp = 0;
    }

    if (upperCamera->hasImage() && upperCamera->getTimeStamp() < lastImageTimeStampLL)
      upperCamera->releaseImage();
    if (lowerCamera->hasImage() && lowerCamera->getTimeStamp() < lastImageTimeStampLL)
      lowerCamera->releaseImage();
  }

#endif
}

void CameraProvider::waitForFrameData() {
#ifdef CAMERA_INCLUDED
  if (theInstance)
    theInstance->waitForFrameData2();
#endif
}

MAKE_MODULE(CameraProvider, Cognition Infrastructure)
