/**
 * @file Walk2014Modifier.h
 *
 * Declaration of a struct for representing how the gyroBalanceFactors of the Walk2014Generator shall be learned
 *
 * This file is subject to the terms of the BHuman 2019 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Philip Reichenberg
 */

#pragma once
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/walk2014_modifier.hpp"
#endif
STREAMABLE_DECLARE(Walk2014Modifier)

STREAMABLE_ROS(Walk2014Modifier, {
  public :,
  FIELD_WRAPPER(int, 0, nomadz_msgs::msg::Walk2014Modifier::num_of_gyro_peaks, numOfGyroPeaks),
  FIELD_WRAPPER(float, 0, nomadz_msgs::msg::Walk2014Modifier::balance_change_factor, balanceChangeFactor),
});
