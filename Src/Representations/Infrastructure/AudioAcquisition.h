/**
 * @file AudioAcquisition.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/audio_acquisition.hpp"
#endif
STREAMABLE_DECLARE(AudioAcquisition)

STREAMABLE_ROS(AudioAcquisition, {
public:
  inline void reset() { value = 0; }
  , FIELD_WRAPPER_DEFAULT(int, nomadz_msgs::msg::AudioAcquisition::value, value), reset();
});
