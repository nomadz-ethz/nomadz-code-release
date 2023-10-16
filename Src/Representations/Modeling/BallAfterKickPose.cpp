/**
 * @file BallAfterKickPose.cpp
 *
 * Implementation of the BallAfterKickPose's drawing functions
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include "Core/Math/Vector2.h"
#include "BallAfterKickPose.h"
#include "Core/Debugging/DebugDrawings.h"

void BallAfterKickPose::draw() const {
  DECLARE_DEBUG_DRAWING("representation:BallAfterKickPose", "drawingOnField"); // drawing of the ball model
  COMPLEX_DRAWING("representation:BallAfterKickPose", {
    const Vector2<>& pos(position);
    ELLIPSE("representation:BallAfterKickPose",
            pos,
            dev.x,
            dev.y,
            rot + pi_2,
            3, // pen width
            Drawings::ps_solid,
            ColorRGBA(255, 100, 100, 100),
            Drawings::bs_solid,
            ColorRGBA(50, 200, 200, 100));
  });
}