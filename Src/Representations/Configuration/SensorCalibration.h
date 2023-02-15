/**
 * @file SensorCalibration.h
 *
 * Declaration of a class for representing the calibration values of sensors.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/sensor_calibration.hpp"
#endif
STREAMABLE_DECLARE(SensorCalibration)

STREAMABLE_ROS(
  SensorCalibration,
  {
    ,
    FIELD_WRAPPER(
      float, 0, nomadz_msgs::msg::SensorCalibration::acc_x_offset, accXOffset), /**< The correction offset in g. */
    FIELD_WRAPPER(
      float, 1.f, nomadz_msgs::msg::SensorCalibration::acc_x_gain, accXGain), /**< The factor between sensor units and g. */
    FIELD_WRAPPER(
      float, 0, nomadz_msgs::msg::SensorCalibration::acc_y_offset, accYOffset), /**< The correction offset in g. */
    FIELD_WRAPPER(
      float, 1.f, nomadz_msgs::msg::SensorCalibration::acc_y_gain, accYGain), /**< The factor between sensor units and g. */
    FIELD_WRAPPER(
      float, 0, nomadz_msgs::msg::SensorCalibration::acc_z_offset, accZOffset), /**< The correction offset in g. */
    FIELD_WRAPPER(
      float, 1.f, nomadz_msgs::msg::SensorCalibration::acc_z_gain, accZGain), /**< The factor between sensor units and g. */
    FIELD_WRAPPER(float,
                  1.f,
                  nomadz_msgs::msg::SensorCalibration::gyro_x_gain,
                  gyroXGain), /**< The factor between sensor units and g. */
    FIELD_WRAPPER(float,
                  1.f,
                  nomadz_msgs::msg::SensorCalibration::gyro_y_gain,
                  gyroYGain), /**< The factor between sensor units and g. */
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::SensorCalibration::fsr_l_f_l_offset, fsrLFLOffset),
    FIELD_WRAPPER(float, 1.f, nomadz_msgs::msg::SensorCalibration::fsr_l_f_l_gain, fsrLFLGain),
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::SensorCalibration::fsr_l_f_r_offset, fsrLFROffset),
    FIELD_WRAPPER(float, 1.f, nomadz_msgs::msg::SensorCalibration::fsr_l_f_r_gain, fsrLFRGain),
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::SensorCalibration::fsr_l_b_l_offset, fsrLBLOffset),
    FIELD_WRAPPER(float, 1.f, nomadz_msgs::msg::SensorCalibration::fsr_l_b_l_gain, fsrLBLGain),
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::SensorCalibration::fsr_l_b_r_offset, fsrLBROffset),
    FIELD_WRAPPER(float, 1.f, nomadz_msgs::msg::SensorCalibration::fsr_l_b_r_gain, fsrLBRGain),
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::SensorCalibration::fsr_r_f_l_offset, fsrRFLOffset),
    FIELD_WRAPPER(float, 1.f, nomadz_msgs::msg::SensorCalibration::fsr_r_f_l_gain, fsrRFLGain),
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::SensorCalibration::fsr_r_f_r_offset, fsrRFROffset),
    FIELD_WRAPPER(float, 1.f, nomadz_msgs::msg::SensorCalibration::fsr_r_f_r_gain, fsrRFRGain),
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::SensorCalibration::fsr_r_b_l_offset, fsrRBLOffset),
    FIELD_WRAPPER(float, 1.f, nomadz_msgs::msg::SensorCalibration::fsr_r_b_l_gain, fsrRBLGain),
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::SensorCalibration::fsr_r_b_r_offset, fsrRBROffset),
    FIELD_WRAPPER(float, 1.f, nomadz_msgs::msg::SensorCalibration::fsr_r_b_r_gain, fsrRBRGain),
  });
