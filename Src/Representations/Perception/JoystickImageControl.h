/**
 * @file JoystickImageControl.h
 *
 * Declaration of a class representing settings for an automated image and
 * camera matrix acquisition.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/joystick_image_control.hpp"
#endif
STREAMABLE_DECLARE(JoystickImageControl)

STREAMABLE_ROS(JoystickImageControl,
               {

                 ,
                 FIELD_WRAPPER(bool, false, nomadz_msgs::msg::JoystickImageControl::upper_image_taken, upperImageTaken),
                 FIELD_WRAPPER(bool, false, nomadz_msgs::msg::JoystickImageControl::lower_image_taken, lowerImageTaken),

               });
