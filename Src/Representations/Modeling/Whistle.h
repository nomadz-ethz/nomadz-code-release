/**
 * @file Whistle.h
 *
 * Identified whistle sound
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/whistle.hpp"
#endif
STREAMABLE_DECLARE(Whistle)

STREAMABLE_ROS(Whistle,
               {
                 ,
                 FIELD_WRAPPER(short,
                               0,
                               nomadz_msgs::msg::Whistle::confidence_of_last_whistle_detection,
                               confidenceOfLastWhistleDetection), /**< Confidence based on hearing capability */
                 FIELD_WRAPPER(unsigned int,
                               0,
                               nomadz_msgs::msg::Whistle::last_time_whistle_detected,
                               lastTimeWhistleDetected), /**< Timestamp */
                 FIELD_WRAPPER(unsigned int,
                               0,
                               nomadz_msgs::msg::Whistle::last_time_of_incoming_sound,
                               lastTimeOfIncomingSound), /**< The last point of time when the robot received audio data */
                 FIELD_WRAPPER(float, 0.0, nomadz_msgs::msg::Whistle::volume, volume),
                 FIELD_WRAPPER(unsigned int,
                               0,
                               nomadz_msgs::msg::Whistle::channel_input,
                               channelInput), /**< The channel the whistle was detected on */
                 FIELD_WRAPPER(bool,
                               false,
                               nomadz_msgs::msg::Whistle::whistle_detected,
                               whistleDetected), /**Boolean for whistle recognition*/
                 FIELD_WRAPPER(float, 0.0, nomadz_msgs::msg::Whistle::whistle_prob_channel_0, wp_zero),
                 FIELD_WRAPPER(float, 0.0, nomadz_msgs::msg::Whistle::whistle_prob_channel_1, wp_one),
                 FIELD_WRAPPER(int, 0, nomadz_msgs::msg::Whistle::buffer_size, bufferSize),
               });
