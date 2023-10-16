/**
 * @file AudioData.h
 *
 * The file declares a struct that stores audio data of up to four channels.
 * On a V4, the four channels are:
 * 0: left microphone
 * 1: right microphone
 * 2: front microphone
 * 3: rear microphone
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include <vector>

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/audio_data.hpp"
#endif
STREAMABLE_DECLARE(AudioData)

STREAMABLE_ROS(AudioData,
               {
                 ,
                 FIELD_WRAPPER(unsigned, 2, nomadz_msgs::msg::AudioData::channels, channels),
                 FIELD_WRAPPER(unsigned, 48000, nomadz_msgs::msg::AudioData::sample_rate, sampleRate),
                 FIELD_WRAPPER_DEFAULT(std::vector<short>,
                                       nomadz_msgs::msg::AudioData::samples,
                                       samples), /**< Samples are interleaved. */
               });
