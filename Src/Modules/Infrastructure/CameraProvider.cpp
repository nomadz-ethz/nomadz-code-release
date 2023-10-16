/**
 * @file CameraProvider.cpp
 *
 * This file declares a module that provides camera images.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#include <cstdio>
#include <unistd.h>

#ifdef ENABLE_ROS
#include <rclcpp/time.hpp>
#endif

#include "CameraProvider.h"
#include "Representations/Perception/JPEGImage.h"
#include "Core/System/SystemCall.h"
#include "Core/System/Time.h"
#include "Core/Streams/InStreams.h"
#include "Core/Debugging/Stopwatch.h"

PROCESS_WIDE_STORAGE(CameraProvider) CameraProvider::theInstance = 0;

CameraProvider::CameraProvider()
#ifndef ENABLE_ROS_CAMERA_NODE
    : currentImageCamera(0)
#ifdef TARGET_ROBOT
      ,
      imageTimeStamp(0), otherImageTimeStamp(0), lastImageTimeStampLL(0), firstImageTimeStamp(0)
#endif
#endif
{
  InMapFile upperStream("upperCameraInfo.cfg");
  ASSERT(upperStream.exists());
  upperStream >> upperCameraInfo;
  InMapFile lowerStream("lowerCameraInfo.cfg");
  ASSERT(lowerStream.exists());
  lowerStream >> lowerCameraInfo;

#ifdef ENABLE_ROS_CAMERA_NODE
  cycleTime = 1.f / 30.f;
#else
#ifdef TARGET_ROBOT
  upperCamera = new NaoCamera("/dev/video-top", upperCameraInfo.camera, upperCameraInfo.width, upperCameraInfo.height, true);
  lowerCamera =
    new NaoCamera("/dev/video-bottom", lowerCameraInfo.camera, lowerCameraInfo.width, lowerCameraInfo.height, false);
  cycleTime = upperCamera->getFrameRate();
  ASSERT(upperCamera->getFrameRate() == lowerCamera->getFrameRate());
#else
  upperCamera = lowerCamera = NULL;
  cycleTime = 1.f / 30.f;
#endif
#endif
  theInstance = this;
}

CameraProvider::~CameraProvider() {
#ifndef ENABLE_ROS_CAMERA_NODE
#ifdef TARGET_ROBOT
  if (upperCamera)
    delete upperCamera;
  if (lowerCamera)
    delete lowerCamera;
#endif
#endif
  theInstance = 0;
}

void CameraProvider::update(Image& image) {
#ifdef ENABLE_ROS_CAMERA_NODE
#ifdef TARGET_ROBOT
  auto set_image_from_msg = [&](const sensor_msgs::msg::Image& ros_image_msg, const CameraInfo& expected_camera_info) {
    if (ros_image_msg.width != expected_camera_info.width * 2 || ros_image_msg.height != expected_camera_info.height * 2) {
      OUTPUT_ERROR("ROS image resolution mismatch. Expected " << expected_camera_info.width * 2 << "x"
                                                              << expected_camera_info.height * 2 << ", got "
                                                              << ros_image_msg.width << "x" << ros_image_msg.height << ".");
      return;
    }

    if (ros_image_msg.encoding != "yuv422_yuy2") {
      OUTPUT_ERROR("ROS image encoding mismatch.");
      return;
    }

    image.setImage(ros_image_msg.data.data());
    image.setResolution(expected_camera_info.width, expected_camera_info.height);
    image.timeStamp = toMillisec(ros_image_msg.header.stamp.sec, ros_image_msg.header.stamp.nanosec);
  };

  if (currentCameraIsUpper) {
    set_image_from_msg(upperImageMsg, upperCameraInfo);
  } else {
    set_image_from_msg(lowerImageMsg, lowerCameraInfo);
  }

  if (image.timeStamp == lastImageTimeStamp) {
    OUTPUT_WARNING("ROSCameraProvider: Image timestamp did not change. Reusing last image.");
    // add a small offset to the timestamp to avoid duplicate timestamps
    lastImageTimeStamp = image.timeStamp + 10;
  } else {
    lastImageTimeStamp = image.timeStamp;
  }
#endif
#else
#ifdef TARGET_ROBOT
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
#endif
#endif
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
#ifdef ENABLE_ROS_CAMERA_NODE
  if (currentCameraIsUpper) {
#else
  if (currentImageCamera == upperCamera) {
#endif
    cameraInfo = upperCameraInfo;
  } else {
    cameraInfo = lowerCameraInfo;
  }
}

bool CameraProvider::isFrameDataComplete() {
#ifdef TARGET_ROBOT
#ifndef ENABLE_ROS_CAMERA_NODE
  if (theInstance)
    return theInstance->upperCamera->hasImage() || theInstance->lowerCamera->hasImage();
  else
#endif
#endif
    return true;
}

void CameraProvider::waitForFrameData() {
#ifdef TARGET_ROBOT
  if (theInstance) {
#ifdef ENABLE_ROS_CAMERA_NODE
    theInstance->pollImageSubscribers();
#else
    theInstance->waitForFrameData2();
#endif
  }
#endif
}

#ifdef ENABLE_ROS_CAMERA_NODE
bool CameraProvider::setupImageSubscribers(rclcpp::Node::SharedPtr ros_node) {
  if (theInstance) {
    OUTPUT_TEXT("Setting up ROS image subscribers");
    theInstance->upperImageSubscriber = ros_node->create_subscription<sensor_msgs::msg::Image>(
      upperImageTopic, 1, [&](const sensor_msgs::msg::Image::SharedPtr msg) {
        assert(false && "Upper image callback should never be called.");
      });
    theInstance->lowerImageSubscriber = ros_node->create_subscription<sensor_msgs::msg::Image>(
      lowerImageTopic, 1, [&](const sensor_msgs::msg::Image::SharedPtr msg) {
        assert(false && "Lower image callback should never be called.");
      });
    return true;
  }
  return false;
}

void CameraProvider::pollImageSubscribers() {
  if (!upperImageSubscriber || !lowerImageSubscriber) {
    OUTPUT_ERROR("ROS image subscribers not set up.");
    return;
  }

  // Poll both image subscribers until either image is updated
  rclcpp::MessageInfo lowerImageMsgInfo;
  rclcpp::MessageInfo upperImageMsgInfo;
  bool receivedUpperImage = false;
  bool receivedLowerImage = false;
  unsigned int upperImageTimestamp = 0;
  unsigned int lowerImageTimestamp = 0;
  unsigned int pollingAttempts = 0;
  for (; pollingAttempts < maxPollingAttempts; ++pollingAttempts) {
    receivedUpperImage = upperImageSubscriber->take(upperImageMsg, upperImageMsgInfo);
    receivedLowerImage = lowerImageSubscriber->take(lowerImageMsg, lowerImageMsgInfo);
    if (receivedUpperImage) {
      upperImageTimestamp = toMillisec(upperImageMsg.header.stamp.sec, upperImageMsg.header.stamp.nanosec);
      if (upperImageTimestamp < lastImageTimeStamp) {
        receivedUpperImage = false;
      }
    }
    if (receivedLowerImage) {
      lowerImageTimestamp = toMillisec(lowerImageMsg.header.stamp.sec, lowerImageMsg.header.stamp.nanosec);
      if (lowerImageTimestamp < lastImageTimeStamp) {
        receivedLowerImage = false;
      }
    }
    if (receivedUpperImage || receivedLowerImage) {
      break;
    }
    
    SystemCall::sleep(1);
  }

  if (pollingAttempts == maxPollingAttempts) {
    OUTPUT_ERROR("ROS image polling timeout.");
    return;
  }

  // now pick between the upper and lower image
  if (receivedUpperImage && (!receivedLowerImage || lowerImageTimestamp < upperImageTimestamp)) {
    currentCameraIsUpper = true;
  } else if (receivedLowerImage) {
    currentCameraIsUpper = false;
  }
}
#else

void CameraProvider::waitForFrameData2() {
#ifdef TARGET_ROBOT

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

    // always apply a hard reset on the v6 as otherwise apparent hang
    if (resetUpper || resetLower) {
      OUTPUT_WARNING("Resetting cameras...");
      system("/usr/libexec/reset-cameras.sh toggle");
      // FIXME: we seem to need to sleep around two seconds here for the script to fix, implement this more properly
      sleep(2);
      resetUpper = true;
      resetLower = true;
    }

    if (resetUpper || resetLower) {
      SystemCall::playSound("cameraReset.wav");
    }

    if (resetUpper) {
      delete upperCamera;
      upperCamera =
        new NaoCamera("/dev/video-top", upperCameraInfo.camera, upperCameraInfo.width, upperCameraInfo.height, true);
      imageTimeStamp = 0;
      firstImageTimeStamp = 0;
    }

    if (resetLower) {
      delete lowerCamera;
      lowerCamera =
        new NaoCamera("/dev/video-bottom", lowerCameraInfo.camera, lowerCameraInfo.width, lowerCameraInfo.height, false);
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
#endif

MAKE_MODULE(CameraProvider, Cognition Infrastructure)
