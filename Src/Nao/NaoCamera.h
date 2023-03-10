/**
 * @file NaoCamera.h
 *
 * Interface to a camera of the NAO.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#pragma once

#include "Core/Streams/InStreams.h"
#include "Representations/Configuration/CameraSettings.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"

#define CAMERA_INCLUDED

/**
 * @class NaoCamera
 * Interface to a camera of the NAO
 */
class NaoCamera {
public:
  /**
   * Constructor.
   * @param device The name of the camera device (e.g. /dev/video0).
   * @param camera Whether this is the lower or upper camera.
   * @param width The width of the camera image in pixels. V4L only allows certain values (e.g. 320 or 640)
   * @param height The height of the camera image in pixels. V4L only allows certain values (e.g. 240 or 320)
   * @param flip Whether the image should be flipped
   */
  NaoCamera(const char* device, CameraInfo::Camera camera, int width, int height, bool flip);

  /** Destructor. */
  ~NaoCamera();

  /**
   * Sets the camera control settings to the camera.
   * @param settings The settings.
   */
  void setSettings(const CameraSettings& settings);

  /**
   * Requests the current camera control settings of the camera.
   * @return The settings.
   */
  const CameraSettings& getSettings() const { return settings; };

  /**
   * The method blocks till a new image arrives.
   * @return true (except a not manageable exception occurs)
   */
  bool captureNew();

  /**
   * The method blocks till one of the given cameras returns an image.
   * @param cam1 The first camera
   * @param cam2 The second camera
   * @param timeout The maximum waiting time
   * @param errorCam1 Whether an error occured on cam1
   * @param errorCam2 Whether an error occured on cam2
   */
  static bool captureNew(NaoCamera& cam1, NaoCamera& cam2, int timeout, bool& errorCam1, bool& errorCam2);

  /**
   * Releases an image that has been captured. That way the buffer can be used to capture another image
   */
  void releaseImage();

  /**
   * The last captured image.
   * @return The image data buffer.
   */
  const unsigned char* getImage() const;

  /**
   * Whether an image has been captured.
   * @return true if there is one
   */
  bool hasImage() const;

  /**
   * Timestamp of the last captured image in ??s.
   * @return The timestamp.
   */
  unsigned long long getTimeStamp() const;

  /**
   * Returns the frame rate used by the camera
   * @return The frame rate in frames per second
   */
  float getFrameRate() const;

  /**
   * Returns whether this is the lower orthe  upper camera
   * @return Whether this is the lower or the upper camera
   */
  CameraInfo::Camera getCurrentCamera() { return camera; }

  /**
   * Asserts that the actual camera settings are correct.
   */
  void assertCameraSettings();

  /**
   * Unconditional write of the camera settings
   *
   * @return The time in milliseconds executing this method took.
   */
  unsigned int writeCameraSettings();

  /**
   * Set frame rate in frames per 10 seconds.
   */
  void setFrameRate(unsigned numerator = 1, unsigned denominator = 30);

  /**
   * The time (in _milliseconds_) passed while waiting for a new image.
   */
  unsigned int timeWaitedForLastImage;

private:
  CameraSettings settings;        /**< The camera control settings. */
  CameraSettings appliedSettings; /**< The camera settings that are known to be applied. */

  enum {
    frameBufferCount = 3, /**< Amount of available frame buffers. */
  };

  unsigned int WIDTH;  /**< The width of the yuv 422 image */
  unsigned int HEIGHT; /**< The height of the yuv 422 image */
#ifndef NDEBUG
  unsigned int SIZE; /**< The size of an image in bytes */
#endif
  int fd;                          /**< The file descriptor for the video device. */
  void* mem[frameBufferCount];     /**< Frame buffer addresses. */
  int memLength[frameBufferCount]; /**< The length of each frame buffer. */
  struct v4l2_buffer* buf;         /**< Reusable parameter struct for some ioctl calls. */
  struct v4l2_buffer* currentBuf;  /**< The last dequeued frame buffer. */
  unsigned long long timeStamp;    /**< Timestamp of the last captured image in microseconds. */
  CameraInfo::Camera camera;       /**< The camera accessed by this driver. */
  bool first;                      /**< First image grabbed? */
  unsigned long long
    lastCameraSettingTimestamp; /**< Timestamp from the last time a camera setting is been applied in microseconds. */
  unsigned long long
    cameraSettingApplicationRate; /**< Minimum time in microseconds between too camera setting applications. */

  /**
   * Requests the value of a camera control setting from camera.
   * @param id The setting id.
   * @return The value.
   */
  int getControlSetting(unsigned int id);

  /**
   * Set extension setting for UVC camera
   * @param extension_unit id Id of the extension to set
   * @param control_selector Id of the selector to change
   * @param size Size of the extension data block
   * @param data Data block to update
   */
  bool setExtensionSetting(uint8_t extension_unit_id, uint8_t control_selector, uint16_t size, uint8_t* data);

  /**
   * Set camera register on V6
   * @param address register address
   * @param value register value
   */
  bool setRegister(uint16_t address, uint16_t value);
  /**
   * Get camera register on V6
   * @param address register address
   */
  bool getRegister(uint16_t address);

  /**
   * Sets the value of a camera control setting to camera.
   * @param id The setting id.
   * @param value The value.
   * @return True on success.
   */
  bool setControlSetting(unsigned int id, int value);

  /**
   * Applies a collection of camera control settings.
   *
   * \param list of control settings.
   * \return True if every control setting has been applied, false otherwise.
   */
  bool setControlSettings(std::list<CameraSettings::V4L2Setting> controlsettings,
                          std::list<CameraSettings::V4L2Setting> appliedControlSettings);

  void initOpenVideoDevice(const char* device);
  void initSetImageFormat();
  void initRequestAndMapBuffers();
  void initQueueAllBuffers();
  void initDefaultControlSettings(bool flip);
  void startCapturing();
};
