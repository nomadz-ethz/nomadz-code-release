/**
 * @file FrameInfo.h
 *
 * The file declares a class that contains information on the current frame.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include <climits>
#include "Core/Streams/AutoStreamable.h"

/**
 * @class FrameInfo
 * A class that contains information on the current frame.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/frame_info.hpp"
#endif
STREAMABLE_DECLARE(FrameInfo)

STREAMABLE_ROS(FrameInfo,
{
public:
  /**
  * The method returns the time difference between a given time stamp and the
  * current frame time.
  * @param timeStamp A time stamp, usually in the past.
  * @return The number of ms passed since the given time stamp.
  */
  int getTimeSince(unsigned timeStamp) const {
    if (timeStamp != INT_MAX) {
    return int(time - timeStamp);
}
else {
  return INT_MAX;
}
}
,
  FIELD_WRAPPER(unsigned,
                0,
                nomadz_msgs::msg::FrameInfo::time,
                time), /**< The time stamp of the data processed in the current frame in ms. */
  FIELD_WRAPPER(float, 1, nomadz_msgs::msg::FrameInfo::cycle_time, cycleTime), /**< Length of one cycle in seconds. */
});

/**
 * @class CognitionFrameInfo
 * A class that contains information on the current Cognition frame.
 * This representation is used to track whether camera images are
 * received. In contrast to FrameInfo, it will be transfered to the
 * Motion process.
 */
STREAMABLE_ALIAS(CognitionFrameInfo, FrameInfo, {});
