/**
 * @file BallSpot.h
 *
 * Declaration of a class that represents a spot that might be an indication of a ball.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Core/Math/Vector2.h"
#include "Core/Streams/AutoStreamable.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/ball_spot.hpp"
#endif
STREAMABLE_DECLARE(BallSpot)

/**
 * @class BallSpot
 * A class that represents a spot that might be an indication of a ball.
 */
STREAMABLE_ROS(BallSpot,
{
public:
    BallSpot(int r, int x, int y): BallSpot()
  {
    radius = r;
    position.x = x;
    position.y = y;
}
, FIELD_WRAPPER_DEFAULT(int, nomadz_msgs::msg::BallSpot::radius, radius), // px

  FIELD_WRAPPER_DEFAULT(Vector2<int>, nomadz_msgs::msg::BallSpot::position, position),
});
