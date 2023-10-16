/**
 * @file InertiaSensorData.h
 *
 * Declaration of class InertiaSensorData.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#pragma once

#include "Core/Math/Vector2.h"
#include "Core/Math/Vector3.h"
#include "Core/Math/Vector.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Core/Math/Angle.h"

/**
 * @class InertiaSensorData
 * Encapsulates inertia sensor data.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/inertia_sensor_data.hpp"
#endif
STREAMABLE_DECLARE(InertiaSensorData)

STREAMABLE_ROS(InertiaSensorData, {
  public : enum {off = SensorData::off}, /**< A special value to indicate that the sensor is missing. */
  FIELD_WRAPPER_DEFAULT(Vector3<float>,
                        nomadz_msgs::msg::InertiaSensorData::gyro,
                        gyro), /**< The change in orientation around the x- and y-axis. (in radian/s) */
  FIELD_WRAPPER_DEFAULT(Vector3<float>,
                        nomadz_msgs::msg::InertiaSensorData::acc,
                        acc), /**< The acceleration along the x-, y- and z-axis. (in m/s^2) */
  FIELD_WRAPPER(bool,
                false,
                nomadz_msgs::msg::InertiaSensorData::calibrated,
                calibrated), /**< Whether the inertia sensors are calibrated or not */
  FIELD_WRAPPER_DEFAULT(Vector2<float>,
                        nomadz_msgs::msg::InertiaSensorData::angle,
                        angle), /**< The change in orientation around the x- and y-axis. (in radian/s) */
  // (Vector2<float>)angle,     /**< The orientation of the torso (in rad). */
});
