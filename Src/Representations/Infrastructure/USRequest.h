/**
 * @file USRequest.h
 *
 * This file declares a class that represents a request for controlling
 * which sonar is fired next.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"

/**
 * @class USRequest
 * A class that represents a request for controlling
 * which sonar is fired next.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/us_request.hpp"
#endif
STREAMABLE_DECLARE(USRequest)

STREAMABLE_ROS(USRequest,
               {
                 ,
                 FIELD_WRAPPER(int,
                               -1,
                               nomadz_msgs::msg::USRequest::send_mode,
                               sendMode), /**< The firing mode for sending. -1 -> don't send. */
                 FIELD_WRAPPER(int,
                               -1,
                               nomadz_msgs::msg::USRequest::receive_mode,
                               receiveMode), /**< The firing mode assumed for received readings. -1 -> ignore. */
               });
