/**
 * @file FsrData.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#pragma once

#include "Core/Math/Vector2.h"
#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/fsr_data.hpp"
#endif
STREAMABLE_DECLARE(FsrData)

STREAMABLE_ROS(FsrData,
               {
                 ,
                 FIELD_WRAPPER(bool, true, nomadz_msgs::msg::FsrData::left_foot_contact, leftFootContact),
                 FIELD_WRAPPER(bool, true, nomadz_msgs::msg::FsrData::right_foot_contact, rightFootContact),
                 FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::FsrData::center_of_pressure, centerOfPressure),
               });
