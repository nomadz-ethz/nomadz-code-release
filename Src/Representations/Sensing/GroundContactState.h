/**
 * @file GroundContactState.h
 *
 * Declaration of class GroundContactState.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"

/**
 * @class GroundContactState
 * Describes whether we got contact with ground or not.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/ground_contact_state.hpp"
#endif
STREAMABLE_DECLARE(GroundContactState)

STREAMABLE_ROS(GroundContactState,
               {
                 ,
                 FIELD_WRAPPER(bool,
                               true,
                               nomadz_msgs::msg::GroundContactState::contact,
                               contact), /**< a foot of the robot touches the ground */
                 FIELD_WRAPPER(bool,
                               false,
                               nomadz_msgs::msg::GroundContactState::fsr,
                               fsr), /**< if the source of the data is from the FSR (more certain) */
               });
