/**
 * @file LinePercept.h
 *
 * Representation "LinePercept", which gets updated in the "ObjectPerceptor" module
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Representations/Perception/ShapePercept.h"
#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/line_percept.hpp"
#endif
STREAMABLE_DECLARE(LinePercept)

STREAMABLE_ROS(LinePercept, {
  using PerceptLineSegment = ShapePercept::PerceptLineSegment;
  ,
    FIELD_WRAPPER_DEFAULT(
      std::vector<PerceptLineSegment>, nomadz_msgs::msg::LinePercept::field_line_segments, fieldLineSegments),
    FIELD_WRAPPER(bool, true, nomadz_msgs::msg::LinePercept::activated, activated),
});
