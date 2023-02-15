/**
 * @file FallDownState.h
 *
 * Declaration of class FallDownState
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Enum.h"

/**
 * @class FallDownState
 *
 * A class that represents the current state of the robot's body
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/fall_down_state.hpp"
#endif
STREAMABLE_DECLARE(FallDownState)

STREAMABLE_ROS(FallDownState, {
public:
  ENUM(State, undefined, upright, onGround, staggering, falling);

  ENUM(Direction, none, front, left, back, right);

  ENUM(Sidestate,
       noot, // since "not" is already a keyword...
       leftwards,
       rightwards,
       fallen // robot did not get up since last sideward fall
  );

  /** Debug drawing. */
  void draw() const,
    FIELD_WRAPPER(
      State, undefined, nomadz_msgs::msg::FallDownState::state, state), /**< Current state of the robot's body. */
    FIELD_WRAPPER(Direction,
                  none,
                  nomadz_msgs::msg::FallDownState::direction,
                  direction), /**< The robot is falling / fell into this direction. */
    FIELD_WRAPPER(
      Sidestate, noot, nomadz_msgs::msg::FallDownState::sidewards, sidewards), /**< Did the robot fell sidewards before? */
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::FallDownState::odometry_rotation_offset, odometryRotationOffset),
});
