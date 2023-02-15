/**
 * @file CoordinatedPassRepresentation.h
 *
 * Representation of the module CoordinatedPassProvider
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Math/Vector2.h"

/**
 * @class CoordinatedPassRepresentation
 *
 * Contains all current knowledge about the ball.
 */
#include "Core/Streams/FieldWrapper.h"

STREAMABLE_DECLARE_LEGACY(CoordinatedPassRepresentation)
STREAMABLE_LEGACY(CoordinatedPassRepresentation, {
  public :,
  FIELD_WRAPPER_LEGACY(bool,
                       false,
                       nomadz_msgs::msg::CoordinatedPassRepresentation::found_pass_target,
                       foundPassTarget), // Flag that tell if the algorithm found a pass target or not.

  FIELD_WRAPPER_DEFAULT_LEGACY(Vector2<>,
                               nomadz_msgs::msg::CoordinatedPassRepresentation::pass_coordinate,
                               passCoordinate), // Coordinate of the pass

  FIELD_WRAPPER_LEGACY(float,
                       1000.f,
                       nomadz_msgs::msg::CoordinatedPassRepresentation::distance_pass,
                       distancePass), // Distance Ball Robot (needed for calculate the strength of the kick

  FIELD_WRAPPER_LEGACY(int,
                       0,
                       nomadz_msgs::msg::CoordinatedPassRepresentation::id_receiver_mate,
                       idReceiverMate), // Number of the receiving robot
});
