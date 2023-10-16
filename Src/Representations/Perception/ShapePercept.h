/**
 * @file ShapePercept.h
 *
 * Contains lines and ball-sized circles detected by the ShapePerceptor
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/percept_line_segment.hpp"
#endif
STREAMABLE_DECLARE(PerceptLineSegment)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/shape_percept.hpp"
#endif
STREAMABLE_DECLARE(ShapePercept)

STREAMABLE_ROS(ShapePercept, {
public:
  /**
   * @class Linesegment
   *
   * This class represents a line segment detected in the image.
   */
  STREAMABLE_ROS(
    PerceptLineSegment,
    {
      ,
      FIELD_WRAPPER_DEFAULT(Vector2<int>,
                            nomadz_msgs::msg::PerceptLineSegment::p1,
                            p1), /**< start point of this linesegment in image coordinates */
      FIELD_WRAPPER_DEFAULT(Vector2<int>,
                            nomadz_msgs::msg::PerceptLineSegment::p2,
                            p2), /**< end point of this linesegment in image coordinates */
      FIELD_WRAPPER(
        unsigned short, 0, nomadz_msgs::msg::PerceptLineSegment::votes, votes), /**< votes accumulated by Hough line */
      FIELD_WRAPPER(
        unsigned short, 0, nomadz_msgs::msg::PerceptLineSegment::hits, hits), /**< hits accumulated by segmenter */
    });

  using LineSegment = PerceptLineSegment;
  ,
    FIELD_WRAPPER(unsigned int,
                  0,
                  nomadz_msgs::msg::ShapePercept::cutoff_y,
                  cutoffY), /**< above this line in image, no edges are calculated */
    FIELD_WRAPPER_DEFAULT(std::vector<PerceptLineSegment>, nomadz_msgs::msg::ShapePercept::segments, segments),
});
