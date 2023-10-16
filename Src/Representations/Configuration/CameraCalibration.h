/**
 * @file CameraCalibration.h
 *
 * Declaration of a class for representing the calibration values of the camera.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Core/Math/Vector3.h"
#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/camera_calibration.hpp"
#endif
STREAMABLE_DECLARE(CameraCalibration)

STREAMABLE_ROS(CameraCalibration,
               {
                 ,
                 FIELD_WRAPPER(float,
                               0,
                               nomadz_msgs::msg::CameraCalibration::camera_tilt_correction,
                               cameraTiltCorrection), /**< The correction of the camera tilt angle in radians. */
                 FIELD_WRAPPER(float,
                               0,
                               nomadz_msgs::msg::CameraCalibration::camera_roll_correction,
                               cameraRollCorrection), /**< The correction of the camera roll angle in radians. */
                 FIELD_WRAPPER(float,
                               0,
                               nomadz_msgs::msg::CameraCalibration::camera_pan_correction,
                               cameraPanCorrection), /**< The correction of the camera pan angle in radians. */
                 FIELD_WRAPPER(float,
                               0,
                               nomadz_msgs::msg::CameraCalibration::upper_camera_tilt_correction,
                               upperCameraTiltCorrection), /**< The correction of the camera tilt angle in radians. */
                 FIELD_WRAPPER(float,
                               0,
                               nomadz_msgs::msg::CameraCalibration::upper_camera_roll_correction,
                               upperCameraRollCorrection), /**< The correction of the camera roll angle in radians. */
                 FIELD_WRAPPER(float,
                               0,
                               nomadz_msgs::msg::CameraCalibration::upper_camera_pan_correction,
                               upperCameraPanCorrection), /**< The correction of the camera pan angle in radians. */
                 FIELD_WRAPPER(float,
                               0,
                               nomadz_msgs::msg::CameraCalibration::body_tilt_correction,
                               bodyTiltCorrection), /**< The correction of the body tilt angle in radians. */
                 FIELD_WRAPPER(float,
                               0,
                               nomadz_msgs::msg::CameraCalibration::body_roll_correction,
                               bodyRollCorrection), /**< The correction of the body roll angle in radians. */
                 FIELD_WRAPPER_DEFAULT(Vector3<>,
                                       nomadz_msgs::msg::CameraCalibration::body_translation_correction,
                                       bodyTranslationCorrection), /**< The correction of the body translation in mm. */
               });
