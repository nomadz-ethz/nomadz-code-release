/**
 * @file FallDownState.cpp
 *
 * Implementation of a debug drawing for the FallDownState.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Carsten Könemann
 */

#include "FallDownState.h"
#include "Core/Debugging/DebugDrawings.h"

void FallDownState::draw() const {
  DECLARE_DEBUG_DRAWING("representation:FallDownState", "drawingOnImage");
  // text-shadow for better visibility
  DRAWTEXT("representation:FallDownState", 26, 26, 35, ColorClasses::black, "State: " << getName(state));
  DRAWTEXT("representation:FallDownState", 26, 51, 35, ColorClasses::black, "Direction: " << getName(direction));
  DRAWTEXT("representation:FallDownState", 26, 76, 35, ColorClasses::black, "Sidewards: " << getName(sidewards));
  // text
  DRAWTEXT("representation:FallDownState", 25, 25, 35, ColorClasses::white, "State: " << getName(state));
  DRAWTEXT("representation:FallDownState", 25, 50, 35, ColorClasses::white, "Direction: " << getName(direction));
  DRAWTEXT("representation:FallDownState", 25, 75, 35, ColorClasses::white, "Sidewards: " << getName(sidewards));
}
