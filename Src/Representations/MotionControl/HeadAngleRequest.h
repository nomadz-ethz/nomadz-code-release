/**
 * @file HeadAngleRequest.h
 *
 * This file declares a class that represents a request to set specific angles for the head joint.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Felix Wenk
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"

/**
 * @class HeadAngleRequest
 * A class that represents the requested head angles.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/head_angle_request.hpp"
#endif
STREAMABLE_DECLARE(HeadAngleRequest)

STREAMABLE_ROS(
  HeadAngleRequest,
  {
    ,
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::HeadAngleRequest::pan, pan),   /**< Head pan target angle in radians. */
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::HeadAngleRequest::tilt, tilt), /**< Head tilt target angle in radians. */
    FIELD_WRAPPER(float,
                  1,
                  nomadz_msgs::msg::HeadAngleRequest::speed,
                  speed), /**< Maximum joint speed to reach target angles in radians/s. */
  });
