/**
 * @file BallPercept.h
 *
 * Very simple representation of a seen ball
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Math/Vector2.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/ball_percept.hpp"
#endif
STREAMABLE_DECLARE(BallPercept)

STREAMABLE_ROS(BallPercept, {
public:
  /** Draws the ball*/

  bool ballInUpperCamera; /**< Info if ball has seen in upper camera */

  void draw() const,
    FIELD_WRAPPER_DEFAULT(Vector2<>,
                          nomadz_msgs::msg::BallPercept::position_in_image,
                          positionInImage), /**< The position of the ball in the current image */
    FIELD_WRAPPER(float,
                  0.f,
                  nomadz_msgs::msg::BallPercept::radius_in_image,
                  radiusInImage), /**< The radius of the ball in the current image */
    FIELD_WRAPPER(bool,
                  false,
                  nomadz_msgs::msg::BallPercept::ball_was_seen,
                  ballWasSeen), /**< Indicates, if the ball was seen in the current image. */
    FIELD_WRAPPER_DEFAULT(Vector2<>,
                          nomadz_msgs::msg::BallPercept::relative_position_on_field,
                          relativePositionOnField), /**< Ball position relative to the robot. */
    FIELD_WRAPPER(int,
                  0,
                  nomadz_msgs::msg::BallPercept::time_when_last_seen,
                  timeWhenLastSeen), /**< Time stamp when ball was last seen */
});
