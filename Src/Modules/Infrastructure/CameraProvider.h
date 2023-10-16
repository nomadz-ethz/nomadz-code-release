/**
 * @file CameraProvider.h
 *
 * This file declares a module that provides camera images.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Configuration/CameraSettings.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"

#ifdef ENABLE_ROS_CAMERA_NODE
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#else
class NaoCamera;
#ifdef TARGET_ROBOT
#include "Nao/NaoCamera.h"
#endif
#endif

MODULE(CameraProvider)
REQUIRES(CameraSettings)
REQUIRES(Image)
PROVIDES_WITH_OUTPUT(Image)
PROVIDES_WITH_MODIFY_AND_OUTPUT(FrameInfo)
PROVIDES_WITH_MODIFY(CognitionFrameInfo)
PROVIDES_WITH_MODIFY_AND_OUTPUT(CameraInfo)
END_MODULE

class CameraProvider : public CameraProviderBase {
private:
  static PROCESS_WIDE_STORAGE(CameraProvider)
    theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  CameraInfo upperCameraInfo;
  CameraInfo lowerCameraInfo;

  float cycleTime;
  unsigned int lastImageTimeStamp{0};

#ifdef ENABLE_ROS_CAMERA_NODE
  friend class Cognition;

  static constexpr const char* upperImageTopic = "representations/UpperImage";
  static constexpr const char* lowerImageTopic = "representations/LowerImage";
  static constexpr unsigned int maxPollingAttempts = 240;

  void pollImageSubscribers();

  static bool setupImageSubscribers(std::shared_ptr<rclcpp::Node> node);

  using ImageSubscriberT = rclcpp::Subscription<sensor_msgs::msg::Image>;
  std::shared_ptr<ImageSubscriberT> upperImageSubscriber;
  std::shared_ptr<ImageSubscriberT> lowerImageSubscriber;

  sensor_msgs::msg::Image upperImageMsg;
  sensor_msgs::msg::Image lowerImageMsg;

  bool currentCameraIsUpper{false};
#else
  void waitForFrameData2();

  unsigned int firstImageTimeStamp;
  NaoCamera* upperCamera;
  NaoCamera* lowerCamera;
  NaoCamera* currentImageCamera;
#ifdef TARGET_ROBOT
  unsigned int imageTimeStamp;
  unsigned int otherImageTimeStamp;
  unsigned long long lastImageTimeStampLL;
#endif
#endif

  void update(Image& image);
  void update(FrameInfo& frameInfo);
  void update(CognitionFrameInfo& cognitionFrameInfo);
  void update(CameraInfo& cameraInfo);

public:
  /**
   * Default constructor.
   */
  CameraProvider();

  /**
   * Destructor.
   */
  ~CameraProvider();

  /**
   * The method returns whether a new image is available.
   * @return Is an new image available?
   */
  static bool isFrameDataComplete();

  /**
   * The method waits for a new image.
   */
  static void waitForFrameData();
};
