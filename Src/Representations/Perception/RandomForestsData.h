/**
 * @file RandomForestsData.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "RandomForests.h"
#include "Core/Streams/AutoStreamable.h"
#include "Core/Streams/Streamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/random_forests_data.hpp"
#endif
STREAMABLE_DECLARE(RandomForestsData)

STREAMABLE_DECLARE_LEGACY(RandomForestsLoad)

STREAMABLE_ROS(
  RandomForestsData,
  {

    ,
    FIELD_WRAPPER(unsigned int, 0, nomadz_msgs::msg::RandomForestsData::patch_size, patchSize),
    FIELD_WRAPPER(int, 0, nomadz_msgs::msg::RandomForestsData::y_lower, yLower),
    FIELD_WRAPPER(int, 0, nomadz_msgs::msg::RandomForestsData::y_upper, yUpper),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::RandomForestsData::upper, upper),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::RandomForestsData::th_ball, thBall),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::RandomForestsData::th_robot_body, thRobotBody),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::RandomForestsData::th_line, thLine),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::RandomForestsData::min_det_rate_ball, minDetRateBall),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::RandomForestsData::min_det_rate_robot_body, minDetRateRobotBody),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::RandomForestsData::min_det_rate_line, minDetRateLine),

  });

// name: used to refer to a particular forest programmatically
// folder: something that indicates where the files reside on disk
// labels: vector of labels, mapping [0, 1, 2, ...] to a RandomForestLabel
STREAMABLE_LEGACY(RandomForestsLoad,
                  {
                    ,
                    FIELD_WRAPPER_DEFAULT_LEGACY(std::string, nomadz_msgs::msg::RandomForestsLoad::name, name),
                    FIELD_WRAPPER_DEFAULT_LEGACY(std::string, nomadz_msgs::msg::RandomForestsLoad::folder, folder),
                    (RandomForests, LabelVector)labels,
                  });
