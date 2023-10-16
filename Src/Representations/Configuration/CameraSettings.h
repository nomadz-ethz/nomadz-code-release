/**
 * @file CameraSettings.h
 *
 * Declaration of a class representing the settings of the PDA camera.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include <list>
#include "Core/Streams/AutoStreamable.h"

/**
 * @class Properties
 * The class represents the properties of the camera.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/v4_l2_setting.hpp"
#endif
STREAMABLE_DECLARE(V4L2Setting)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/camera_settings.hpp"
#endif
STREAMABLE_DECLARE(CameraSettings)

STREAMABLE_ROS(CameraSettings, {
private:
  /**
   * Initializes everything with invalid values except the settings for auto features.
   * The settings for auto features are initialized so that they disable the
   * features by default.
   */
  void init();

public:
  STREAMABLE_ROS(V4L2Setting, {
  public:
    inline V4L2Setting(int command, int value);
    bool operator==(const V4L2Setting& o) const { return command == o.command && value == o.value; }
    bool operator!=(const V4L2Setting& o) const { return !(*this == o); }

    int command, FIELD_WRAPPER(int, 0, nomadz_msgs::msg::V4L2Setting::value, value), // only stream value
  });

  bool operator==(const CameraSettings& o) const;
  bool operator!=(const CameraSettings& o) const { return !(*this == o); }
  std::list<V4L2Setting> getChangesAndAssign(const CameraSettings& other);
  std::list<V4L2Setting> getSettings() const;
  void setSetting(const V4L2Setting& setting);

  static const int numSettings = 10,
                   FIELD_WRAPPER_DEFAULT(V4L2Setting,
                                         nomadz_msgs::msg::CameraSettings::auto_white_balance,
                                         autoWhiteBalance), /* 1: Use auto white balance, 0: disable auto white balance. */
    // (V4L2Setting)autoExposure,                    /* 1: Use auto exposure, 0: disable auto exposure. */
    FIELD_WRAPPER_DEFAULT(V4L2Setting, nomadz_msgs::msg::CameraSettings::auto_exposure, autoExposure),
                   FIELD_WRAPPER_DEFAULT(V4L2Setting,
                                         nomadz_msgs::msg::CameraSettings::contrast,
                                         contrast), /* The contrast in range of [0 .. 127] */
    FIELD_WRAPPER_DEFAULT(
      V4L2Setting, nomadz_msgs::msg::CameraSettings::saturation, saturation), /* The saturation in range of [0 .. 255] */
    FIELD_WRAPPER_DEFAULT(V4L2Setting, nomadz_msgs::msg::CameraSettings::hue, hue), /* The hue in range [-180 .. 180] */
    FIELD_WRAPPER_DEFAULT(
      V4L2Setting, nomadz_msgs::msg::CameraSettings::sharpness, sharpness), /* The sharpness in range of [0 .. 31] */
    FIELD_WRAPPER_DEFAULT(V4L2Setting,
                          nomadz_msgs::msg::CameraSettings::exposure,
                          exposure), /**< The exposure time in the range of [0 .. 1023]. */
    FIELD_WRAPPER_DEFAULT(
      V4L2Setting, nomadz_msgs::msg::CameraSettings::gain, gain), /**< The gain level in the range of [0 .. 127]. */
    FIELD_WRAPPER_DEFAULT(V4L2Setting,
                          nomadz_msgs::msg::CameraSettings::white_balance,
                          whiteBalance), /**< The white balance in Kelvin [2700 .. 6500] */
    FIELD_WRAPPER_DEFAULT(V4L2Setting,
                          nomadz_msgs::msg::CameraSettings::fade_to_black,
                          fadeToBlack), /**< Fade to black under low light conditions */
    // Initialization
    init();
});

CameraSettings::V4L2Setting::V4L2Setting(int command, int value) : command(command), value(value) {}
