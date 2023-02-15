/**
 * @file KeyStates.h
 *
 * Declaration of class KeyStates
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Enum.h"

/**
 * The class represents the states of the keys.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/key_states.hpp"
#endif
STREAMABLE_DECLARE(KeyStates)

STREAMABLE_ROS(KeyStates, {
public:
  ENUM(Key, rightFootRight, rightFootLeft, leftFootRight, leftFootLeft, chest, headFront, headMiddle, headRear)
  , FIELD_WRAPPER_DEFAULT(bool[numOfKeys], nomadz_msgs::msg::KeyStates::pressed, pressed),

    // Initialization
    for (int i = 0; i < numOfKeys; ++i) pressed[i] = false;
});
