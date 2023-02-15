/**
 * @file LEDRequest.h
 *
 * This file contains the LEDRequest class.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Enum.h"

/**
 * This describes a LEDRequest
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/led_request.hpp"
#endif
STREAMABLE_DECLARE(LEDRequest)

STREAMABLE_ROS(LEDRequest, {
public:
  /** ids for all LEDs */
  ENUM(LED,
       faceLeftRed0Deg,
       faceLeftRed45Deg,
       faceLeftRed90Deg,
       faceLeftRed135Deg,
       faceLeftRed180Deg,
       faceLeftRed225Deg,
       faceLeftRed270Deg,
       faceLeftRed315Deg,
       faceLeftGreen0Deg,
       faceLeftGreen45Deg,
       faceLeftGreen90Deg,
       faceLeftGreen135Deg,
       faceLeftGreen180Deg,
       faceLeftGreen225Deg,
       faceLeftGreen270Deg,
       faceLeftGreen315Deg,
       faceLeftBlue0Deg,
       faceLeftBlue45Deg,
       faceLeftBlue90Deg,
       faceLeftBlue135Deg,
       faceLeftBlue180Deg,
       faceLeftBlue225Deg,
       faceLeftBlue270Deg,
       faceLeftBlue315Deg,
       faceRightRed0Deg,
       faceRightRed45Deg,
       faceRightRed90Deg,
       faceRightRed135Deg,
       faceRightRed180Deg,
       faceRightRed225Deg,
       faceRightRed270Deg,
       faceRightRed315Deg,
       faceRightGreen0Deg,
       faceRightGreen45Deg,
       faceRightGreen90Deg,
       faceRightGreen135Deg,
       faceRightGreen180Deg,
       faceRightGreen225Deg,
       faceRightGreen270Deg,
       faceRightGreen315Deg,
       faceRightBlue0Deg,
       faceRightBlue45Deg,
       faceRightBlue90Deg,
       faceRightBlue135Deg,
       faceRightBlue180Deg,
       faceRightBlue225Deg,
       faceRightBlue270Deg,
       faceRightBlue315Deg,
       earsLeft0Deg,
       earsLeft36Deg,
       earsLeft72Deg,
       earsLeft108Deg,
       earsLeft144Deg,
       earsLeft180Deg,
       earsLeft216Deg,
       earsLeft252Deg,
       earsLeft288Deg,
       earsLeft324Deg,
       earsRight0Deg,
       earsRight36Deg,
       earsRight72Deg,
       earsRight108Deg,
       earsRight144Deg,
       earsRight180Deg,
       earsRight216Deg,
       earsRight252Deg,
       earsRight288Deg,
       earsRight324Deg,
       chestRed,
       chestGreen,
       chestBlue,
       footLeftRed,
       footLeftGreen,
       footLeftBlue,
       footRightRed,
       footRightGreen,
       footRightBlue);

  ENUM(LEDState, off, on, blinking, fastBlinking, half);

  bool operator==(const LEDRequest& other) const {
    for (int i = 0; i < numOfLEDs; i++)
      if (ledStates[i] != other.ledStates[i])
        return false;
    return true;
  }

  bool operator!=(const LEDRequest& other) const { return !(*this == other); }
  ,
    FIELD_WRAPPER_DEFAULT(LEDState[numOfLEDs],
                          nomadz_msgs::msg::LEDRequest::led_states,
                          ledStates), /**< The intended states of the LEDs (use type State). */

    // Initialization
    for (int i = 0; i < numOfLEDs; ++i) ledStates[i] = off;
});
