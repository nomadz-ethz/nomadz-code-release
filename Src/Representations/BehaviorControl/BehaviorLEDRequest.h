/**
 * @file BehaviorLEDRequest.h
 *
 * This file contains the BehaviorLEDRequest class.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 */

#pragma once

#include "Representations/Infrastructure/LEDRequest.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/behavior_led_request.hpp"
#endif
STREAMABLE_DECLARE(BehaviorLEDRequest)

STREAMABLE_ROS(BehaviorLEDRequest, {
public:
  ENUM(BehaviorLED, leftEye, rightEye, leftEar, rightEar);

  ENUM(EyeColor, defaultColor, red, green, blue, white, magenta, yellow, cyan);

  // FIXME: Copy from LEDRequest
  ENUM_ALIAS(LEDRequest, LEDState, off, on, blinking, fastBlinking, half);

  bool operator==(const BehaviorLEDRequest& other) const {
    for (int i = 0; i < numOfBehaviorLEDs; i++)
      if (modifiers[i] != other.modifiers[i])
        return false;
    return true;
  }

  bool operator!=(const BehaviorLEDRequest& other) const { return !(*this == other); }
  ,
    FIELD_WRAPPER_DEFAULT(
      LEDRequest::LEDState[numOfBehaviorLEDs], nomadz_msgs::msg::BehaviorLEDRequest::modifiers, modifiers),
    FIELD_WRAPPER(EyeColor, defaultColor, nomadz_msgs::msg::BehaviorLEDRequest::left_eye_color, leftEyeColor),
    FIELD_WRAPPER(EyeColor, defaultColor, nomadz_msgs::msg::BehaviorLEDRequest::right_eye_color, rightEyeColor),

    // Initialization
    for (int i = 0; i < numOfBehaviorLEDs; ++i) modifiers[i] = LEDRequest::on;
});
