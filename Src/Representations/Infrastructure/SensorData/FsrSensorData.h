/**
 * @file FsrSensorData.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2023 Nao Devils and NomadZ team
 */

#pragma once

#include "Core/SensorData.h"
#include "Core/Streams/AutoStreamable.h"

#include <array>

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/fsr_sensor_data.hpp"
#endif
STREAMABLE_DECLARE(FsrSensorData)

STREAMABLE_ROS(FsrSensorData, {
public:
  ENUM(FsrSensorPosition, fl, fr, bl, br);
  ,
    FIELD_WRAPPER_DEFAULT(float[FsrSensorData::numOfFsrSensorPositions],
                          nomadz_msgs::msg::FsrSensorData::left,
                          left), /**< Values of the four pressure sensors in the left foot (in kg) */
    FIELD_WRAPPER_DEFAULT(float[FsrSensorData::numOfFsrSensorPositions],
                          nomadz_msgs::msg::FsrSensorData::right,
                          right), /**< Values of the four pressure sensors in the right foot (in kg) */
    FIELD_WRAPPER(float,
                  0.f,
                  nomadz_msgs::msg::FsrSensorData::left_total,
                  leftTotal), /**< Total mass pressing on the left foot (in kg) */
    FIELD_WRAPPER(float,
                  0.f,
                  nomadz_msgs::msg::FsrSensorData::right_total,
                  rightTotal), /**< Total mass pressing on the right foot (in kg) */
                               // Initialization
    for (int i = 0; i < FsrSensorData::numOfFsrSensorPositions; ++i) left[i] = SensorOff::off;
  for (int j = 0; j < FsrSensorData::numOfFsrSensorPositions; ++j)
    right[j] = SensorOff::off;
});
