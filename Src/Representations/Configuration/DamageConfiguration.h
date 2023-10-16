/**
 * @file DamageConfiguration.h
 *
 * Provides data about disabling some functions because of hardware failures.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Benjamin Markowsky
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Math/Vector2.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/damage_configuration.hpp"
#endif
STREAMABLE_DECLARE(DamageConfiguration)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/damage_configuration_head.hpp"
#endif
STREAMABLE_DECLARE(DamageConfigurationHead)

STREAMABLE_ROS(
  DamageConfiguration,
  {
    ,
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::DamageConfiguration::weak_left_leg, weakLeftLeg),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::DamageConfiguration::weak_right_leg, weakRightLeg),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::DamageConfiguration::us_l_defect, usLDefect),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::DamageConfiguration::us_r_defect, usRDefect),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::DamageConfiguration::left_foot_bumper_defect, leftFootBumperDefect),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::DamageConfiguration::right_foot_bumper_defect, rightFootBumperDefect),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::DamageConfiguration::dont_boost, dontBoost),
    FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::DamageConfiguration::start_tilt_left, startTiltLeft),
    FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::DamageConfiguration::start_tilt_right, startTiltRight),
  });

STREAMABLE_ROS(
  DamageConfigurationHead,
  {
    ,
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::DamageConfigurationHead::audio_channel0_defect, audioChannel0Defect),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::DamageConfigurationHead::audio_channel1_defect, audioChannel1Defect),
  });
