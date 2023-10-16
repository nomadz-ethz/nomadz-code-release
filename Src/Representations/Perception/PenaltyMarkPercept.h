/**
 * @file LinePercept.h
 *
 * Declaration of the representation PenaltyMarkPercept
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Debugging/DebugDrawings.h"
#include "Core/Math/Vector2.h"
#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/penalty_mark_percept.hpp"
#endif
STREAMABLE_DECLARE(PenaltyMarkPercept)

STREAMABLE_ROS(PenaltyMarkPercept, {
public:
  void draw() const {
    DECLARE_DEBUG_DRAWING("representation:PenaltyMarkPercept:Image", "drawingOnImage");
    DECLARE_DEBUG_DRAWING("representation:PenaltyMarkPercept:Field", "drawingOnField");

    if (seen) {
  CROSS("representation:PenaltyMarkPercept:Image",
        positionInImage.x,
        positionInImage.y,
        5,
        2,
        Drawings::ps_solid,
        ColorClasses::blue);
  CROSS("representation:PenaltyMarkPercept:Field",
        relativePositionOnField.x,
        relativePositionOnField.y,
        50,
        25,
        Drawings::ps_solid,
        ColorClasses::blue);
    }
}
, FIELD_WRAPPER(bool, false, nomadz_msgs::msg::PenaltyMarkPercept::seen, seen),
  FIELD_WRAPPER_DEFAULT(Vector2<int>, nomadz_msgs::msg::PenaltyMarkPercept::position_in_image, positionInImage),
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::PenaltyMarkPercept::relative_position_on_field,
                        relativePositionOnField),
});
