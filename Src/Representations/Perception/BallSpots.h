/**
 * @file BallSpots.h
 *
 * Declaration of a class that represents a spot that might be an indication of a ball.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:jworch@informatik.uni-bremen.de">Jan-Hendrik Worch</a>
 * are <a href="mailto:ingsie@informatik.uni-bremen.de">Ingo Sieverdingbeck</a>
 */

#pragma once

#include "Core/Debugging/DebugDrawings.h"
#include "Core/Math/Vector2.h"
#include "BallSpot.h"

/**
 * @class BallSpots
 * A class that represents a spot that might be an indication of a ball.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/ball_spots.hpp"
#endif
STREAMABLE_DECLARE(BallSpots)

STREAMABLE_ROS(BallSpots, {
public:
  void addBallSpot(int x, int y) {
    BallSpot bs;
    bs.position.x = x;
    bs.position.y = y;
    ballSpots.push_back(bs);
  }

  void addBallSpot(int r, int x, int y) { ballSpots.push_back(BallSpot(r, x, y)); }

  /** The method draws all ball spots. */
  void draw() const {
    DECLARE_DEBUG_DRAWING("representation:BallSpots:Image", "drawingOnImage"); // Draws the ballspots to the image
    COMPLEX_DRAWING("representation:BallSpots:Image", {
      for (std::vector<BallSpot>::const_iterator i = ballSpots.begin(); i != ballSpots.end(); ++i) {
        CROSS(
          "representation:BallSpots:Image", i->position.x, i->position.y, 2, 3, Drawings::ps_solid, ColorClasses::orange);
        CROSS("representation:BallSpots:Image", i->position.x, i->position.y, 2, 0, Drawings::ps_solid, ColorClasses::black);
      }
    });
  }
  , FIELD_WRAPPER_DEFAULT(std::vector<BallSpot>, nomadz_msgs::msg::BallSpots::ball_spots, ballSpots),

    // Initialization
    ballSpots.reserve(50);
});
