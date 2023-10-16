/**
 * @file HeadJointRequest.h
 *
 * This file declares a class that represents the requested head joint angles.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"

/**
 * @class HeadJointRequest
 * A class that represents the requested head joint angles.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/head_joint_request.hpp"
#endif
STREAMABLE_DECLARE(HeadJointRequest)

STREAMABLE_ROS(
  HeadJointRequest,
  {
    ,
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::HeadJointRequest::pan, pan),   /**< Head pan target angle in radians. */
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::HeadJointRequest::tilt, tilt), /**< Head tilt target angle in radians. */
    FIELD_WRAPPER(bool,
                  true,
                  nomadz_msgs::msg::HeadJointRequest::reachable,
                  reachable), /**< Whether the head motion request points on a reachable position. */
    FIELD_WRAPPER(bool,
                  false,
                  nomadz_msgs::msg::HeadJointRequest::moving,
                  moving), /**< Whether the head is currently in motion or not. */
  });
