/**
 * @file JoyStickControl.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include <string>

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/joystick_control.hpp"
#endif
STREAMABLE_DECLARE(JoystickControl)

STREAMABLE_ROS(JoystickControl,
               {

                 ,
                 FIELD_WRAPPER(bool, false, nomadz_msgs::msg::JoystickControl::acquire_data_upper, acquireDataUpper),
                 FIELD_WRAPPER(bool, false, nomadz_msgs::msg::JoystickControl::acquire_data_lower, acquireDataLower),
                 FIELD_WRAPPER(bool, false, nomadz_msgs::msg::JoystickControl::print_outs, printOuts),
                 FIELD_WRAPPER_DEFAULT(std::string, nomadz_msgs::msg::JoystickControl::path_data, pathData),

               });
