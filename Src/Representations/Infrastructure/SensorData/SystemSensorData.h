/**
 * @file SystemSensorData.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2023 Nao Devils and NomadZ team
 */

#pragma once

#include "Core/SensorData.h"
#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/system_sensor_data.hpp"
#endif
STREAMABLE_DECLARE(SystemSensorData)

STREAMABLE_ROS(
  SystemSensorData,
  {
    ,
    FIELD_WRAPPER(float,
                  SensorData::off,
                  nomadz_msgs::msg::SystemSensorData::cpu_temperature,
                  cpuTemperature), /** The temperatur of the cpu (in C). */
    FIELD_WRAPPER(float,
                  SensorData::off,
                  nomadz_msgs::msg::SystemSensorData::battery_current,
                  batteryCurrent), /** The current of the battery (in A). */
    FIELD_WRAPPER(float,
                  SensorData::off,
                  nomadz_msgs::msg::SystemSensorData::battery_level,
                  batteryLevel), /** The current of the battery (in %). Range: [0.0, 1.0] */
    FIELD_WRAPPER(float,
                  SensorData::off,
                  nomadz_msgs::msg::SystemSensorData::battery_temperature,
                  batteryTemperature), /** The temperatur of the battery (in %, whatever that means...). Range: [0.0, 1.0] */
  });
