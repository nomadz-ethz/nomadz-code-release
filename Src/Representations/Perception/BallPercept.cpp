/**
 * @file BallPercept.cpp
 *
 * Very simple representation of a seen ball
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#include "BallPercept.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Debugging/DebugDrawings3D.h"

void BallPercept::draw() const {
  DECLARE_DEBUG_DRAWING("representation:BallPercept:Image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:BallPercept:Field", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:BallPercept", "origin");
  TRANSLATE3D("representation:BallPercept", 0, 0, -230);
  if (ballWasSeen) {
    CIRCLE("representation:BallPercept:Image",
           positionInImage.x,
           positionInImage.y,
           radiusInImage,
           1, // pen width
           Drawings::ps_solid,
           ColorClasses::black,
           Drawings::bs_solid,
           ColorRGBA(255, 128, 64, 100));
    CIRCLE("representation:BallPercept:Field",
           relativePositionOnField.x,
           relativePositionOnField.y,
           35,
           0, // pen width
           Drawings::ps_solid,
           ColorClasses::orange,
           Drawings::bs_null,
           ColorClasses::orange);
    // Sorry, no access to field dimensions here, so ball radius is hard coded
    SPHERE3D(
      "representation:BallPercept", relativePositionOnField.x, relativePositionOnField.y, 35, 35, ColorClasses::orange);
  }
}
