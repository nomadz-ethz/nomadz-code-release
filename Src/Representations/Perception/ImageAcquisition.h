/**
 * @file ImageAcquisition.h
 *
 * Declaration of a class representing settings for an automated image and
 * camera matrix acquisition.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/image_acquisition.hpp"
#endif
STREAMABLE_DECLARE(ImageAcquisition)

STREAMABLE_ROS(ImageAcquisition, {
public:
  ENUM(Camera, upper, lower, both, JerseyNRBased);
  ENUM(ColorSpace, YCrCb, RGB);
  , FIELD_WRAPPER(bool, false, nomadz_msgs::msg::ImageAcquisition::activated, activated),
    FIELD_WRAPPER(Camera, upper, nomadz_msgs::msg::ImageAcquisition::selected_camera, selectedCamera),
    FIELD_WRAPPER(ColorSpace, YCrCb, nomadz_msgs::msg::ImageAcquisition::color_space, colorSpace),
    FIELD_WRAPPER(unsigned int,
                  2000,
                  nomadz_msgs::msg::ImageAcquisition::acquisition_rate_in_milliseconds,
                  acquisitionRateInMilliseconds),
    FIELD_WRAPPER(bool, true, nomadz_msgs::msg::ImageAcquisition::save_camera_matrix, saveCameraMatrix),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::ImageAcquisition::save_ball_patches, saveBallPatches),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::ImageAcquisition::obtain_r_f_data, obtainRFData),
});
