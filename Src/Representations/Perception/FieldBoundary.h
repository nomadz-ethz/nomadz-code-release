/**
 * @file FieldBoundary.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Alexis Tsogias
 */

#pragma once

#include <vector>

#include "Core/Math/Vector2.h"
#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/field_boundary.hpp"
#endif
STREAMABLE_DECLARE(FieldBoundary)

STREAMABLE_ROS(FieldBoundary, {
public:
  typedef std::vector<Vector2<int>> InImage;   ///< Type for the boundary in image coordinates.
  typedef std::vector<Vector2<float>> InField; ///< Type for the boundary in field coordinates.

  std::vector<InImage> convexBoundaryCandidates; ///< Possible bondary candidates.

  /**
   * Draws some DebugDrawings
   *
   * Modifiable:
   *   representation:FieldBoundary:SelectedCandidate - Set the candidate which shall be drawn
   *
   * DebugDrawings:
   *   representation:FieldBoundary:BoundarySpots
   *   representation:FieldBoundary:ConvexBoundary
   *   representation:FieldBoundary:BoundaryCandidates
   *   representation:FieldBoundary:Image - The field final boundary in the image.
   *   representation:FieldBoundary:Field - The field final boundary on the field.
   *   representation:FieldBoundary:HighestPoint
   */
  void draw() const;

  /**
   * Returns the y coordinate of the field boundary at the specified x coordiante in the current image.
   */
  int getBoundaryY(int x) const,
    FIELD_WRAPPER_DEFAULT(
      std::vector<Vector2<int>>, nomadz_msgs::msg::FieldBoundary::boundary_spots, boundarySpots), ///< Spots on the boundary.

    FIELD_WRAPPER_DEFAULT(
      std::vector<Vector2<int>>,
      nomadz_msgs::msg::FieldBoundary::convex_boundary,
      convexBoundary), ///< A convex upper hull arround the spots that schould fit best the actual boundary.

    FIELD_WRAPPER_DEFAULT(std::vector<Vector2<float>>,
                          nomadz_msgs::msg::FieldBoundary::boundary_on_field,
                          boundaryOnField), ///< The boundary projectet to the Field in relative coordinates.

    FIELD_WRAPPER_DEFAULT(std::vector<Vector2<int>>,
                          nomadz_msgs::msg::FieldBoundary::boundary_in_image,
                          boundaryInImage), ///< The boundary in image coordinates.

    FIELD_WRAPPER_DEFAULT(
      Vector2<int>, nomadz_msgs::msg::FieldBoundary::highest_point, highestPoint), ///< The highest pont of the boundary.

    FIELD_WRAPPER(
      bool, false, nomadz_msgs::msg::FieldBoundary::is_valid, isValid), ///< True if a boundary could be detected.

    FIELD_WRAPPER(int,
                  16,
                  nomadz_msgs::msg::FieldBoundary::scanline_distance,
                  scanlineDistance), ///< The distance between the scanlines used to find the boundarySpots.

    FIELD_WRAPPER(int,
                  0,
                  nomadz_msgs::msg::FieldBoundary::width,
                  width), ///< The width of the current image; used for some debug drawings.
});
