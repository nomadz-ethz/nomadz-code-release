/**
 * @file LineAnalysis.h
 *
 * Declaration of a class that represents the fieldline percepts
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include <list>

/**
 * @class LineAnalysis
 * A class that represents the found fieldlines, center circle and intersections.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/line_segment.hpp"
#endif
STREAMABLE_DECLARE(LineSegment)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/observed_field_line.hpp"
#endif
STREAMABLE_DECLARE(ObservedFieldLine)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/circle_spot.hpp"
#endif
STREAMABLE_DECLARE(CircleSpot)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/intersection.hpp"
#endif
STREAMABLE_DECLARE(Intersection)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/penalty_area_side.hpp"
#endif
STREAMABLE_DECLARE(PenaltyAreaSide)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/line_analysis.hpp"
#endif
STREAMABLE_DECLARE(LineAnalysis)

STREAMABLE_ROS(LineAnalysis, {
public:
  /**
   * @class Linesegment
   *
   * This class represents a linesegment generated from a lienspot.
   */
  STREAMABLE_ROS(LineSegment, {
    public :,
    FIELD_WRAPPER(float,
                  0.f,
                  nomadz_msgs::msg::LineSegment::alpha,
                  alpha), /**< direction of representation in Hesse norm form of this linesegment */
    FIELD_WRAPPER(float,
                  0.f,
                  nomadz_msgs::msg::LineSegment::d,
                  d), /**< distance of representation in Hesse norm form of this linesegment */
    FIELD_WRAPPER_DEFAULT(
      Vector2<>, nomadz_msgs::msg::LineSegment::p1, p1), /**< start point of this linesegment in field coordinates */
    FIELD_WRAPPER_DEFAULT(
      Vector2<>, nomadz_msgs::msg::LineSegment::p2, p2), /**< end point of this linesegment in field coordinates */
    FIELD_WRAPPER_DEFAULT(Vector2<int>,
                          nomadz_msgs::msg::LineSegment::p1_img,
                          p1Img), /**< start point of this linesegment in image coordinates */
    FIELD_WRAPPER_DEFAULT(
      Vector2<int>, nomadz_msgs::msg::LineSegment::p2_img, p2Img), /**< end point of this linesegment in image coordinates */
  });

  /**
   * @class Line
   *
   * This class represents a found fieldline.
   */
  STREAMABLE_ROS(ObservedFieldLine, {
  public:
    /**
     * Calculates the distance of a point p the this line
     * @param p a point
     * @return the distance
     */
    float calculateDistToLine(const Vector2<>& p) const
    {
      return p.x * std::cos(alpha) + p.y * std::sin(alpha) - d;
    }

    /**
     * Calculates the closest point to a point on this line
     * @param p a point
     * @return the closest point on this line
     */
    Vector2<> calculateClosestPointOnLine(const Vector2<>& p) const,
FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::ObservedFieldLine::alpha, alpha), /**< direction of this line in Hesse norm form */
FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::ObservedFieldLine::d, d), /**< distance of this line in Hesse norm form */
FIELD_WRAPPER(bool, false, nomadz_msgs::msg::ObservedFieldLine::dead, dead), /**< This is needed for merging lines */
FIELD_WRAPPER(bool, false, nomadz_msgs::msg::ObservedFieldLine::mid_line, midLine), /**< Whether this is the line throught the center circle */
FIELD_WRAPPER_DEFAULT(std::vector<LineSegment>, nomadz_msgs::msg::ObservedFieldLine::segments, segments), /**< The linesegments forming this line */
FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::ObservedFieldLine::first, first), /**< The starting point of this line in field coordinates */
FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::ObservedFieldLine::last, last), /**< The end point of this line in field coordinates */
FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::ObservedFieldLine::start_in_image, startInImage), /**< The start point of this line in image coordinates */
FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::ObservedFieldLine::end_in_image, endInImage), /**< The end point of this line in image coordinates */
});

/**
 * @class CircleSpot
 *
 * This class represents circle spots. A circle spot
 * is a point calculated from a linesegment where the
 * center circle would be if the linesegment is part
 * of the center circle.
 * This is also used for the found circle.
 */
STREAMABLE_ROS(CircleSpot, {
  friend class LineAnalyzer;                 // Access to iterator
  std::list<LineSegment>::iterator iterator, /**< An temporary iterator pointing to the according segment
                                                  in the singleSegs list */
    FIELD_WRAPPER_DEFAULT(Vector2<float>,
                          nomadz_msgs::msg::CircleSpot::pos,
                          pos), /**< The position of the center of the center circle in field coordinates */
    FIELD_WRAPPER(
      bool, false, nomadz_msgs::msg::CircleSpot::found, found), /**< Whether the center circle was found in this frame */
    FIELD_WRAPPER(
      unsigned, 0, nomadz_msgs::msg::CircleSpot::last_seen, lastSeen), /**< The last time the center circle was seen */
});

/**
 * @class Intersection
 * A class representing a intersection of two fieldlines
 */
STREAMABLE_ROS(Intersection, {
  public : ENUM(IntersectionType, L, T, X),
  FIELD_WRAPPER(IntersectionType, 0, nomadz_msgs::msg::Intersection::type, type),
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::Intersection::pos,
                        pos), /**< The fieldcoordinates of the intersection */
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::Intersection::dir1,
                        dir1), /**< The first direction of the lines intersected. */
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::Intersection::dir2,
                        dir2), /**< The second direction of the lines intersected. */
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::Intersection::first1,
                        first1), /**< Starting point of line crossing this intersection in direction dir1 */
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::Intersection::last1,
                        last1), /**< Ending point of line crossing this intersection in direction dir1 */
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::Intersection::first2,
                        first2), /**< Starting point of line crossing this intersection in direction dir2 */
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::Intersection::last2,
                        last2), /**< Ending point of line crossing this intersection in direction dir2 */
});

/**
 * @class PenaltyAreaSide
 * A class representing a side of the penalty area, also a T-intersection connected to an L-intersection.
 */
STREAMABLE_ROS(PenaltyAreaSide, {
  public :,
  FIELD_WRAPPER_DEFAULT(Intersection, nomadz_msgs::msg::PenaltyAreaSide::tee, tee), /**< The T intersection on this side */
  FIELD_WRAPPER_DEFAULT(Intersection, nomadz_msgs::msg::PenaltyAreaSide::corner, corner), /**< The L corner on this side */
  FIELD_WRAPPER(bool, false, nomadz_msgs::msg::PenaltyAreaSide::is_left, isLeft), /**< True if left side of penalty area */
});

/** Determines the closest line to a given point
 * @param point the given point
 * @param retLine the closest line
 * @return the distance from point to retLine
 * */
float getClosestLine(Vector2<> point, ObservedFieldLine& retLine) const;

/**
 * The method draws the line percepts on the field.
 */
void drawOnField(const FieldDimensions& theFieldDimensions, float circleBiggerThanSpecified) const;

/**
 * The method draws the line percepts on the image.
 */
void drawOnImage(const CameraMatrix& theCameraMatrix,
                 const CameraInfo& theCameraInfo,
                 const FieldDimensions& theFieldDimensions,
                 float circleBiggerThanSpecified,
                 const ImageCoordinateSystem& theImageCoordinateSystem) const;

/**
 * The method draws the line percepts in the 3D View.
 */
void drawIn3D(const FieldDimensions&theFieldDimensions, float circleBiggerThanSpecified) const,
  FIELD_WRAPPER_DEFAULT(std::vector<ObservedFieldLine>,
                        nomadz_msgs::msg::LineAnalysis::lines,
                        lines), /**< The found fieldlines */
  FIELD_WRAPPER_DEFAULT(std::vector<Intersection>,
                        nomadz_msgs::msg::LineAnalysis::intersections,
                        intersections), /**< The intersections of the lines */
  FIELD_WRAPPER_DEFAULT(std::vector<LineSegment>,
                        nomadz_msgs::msg::LineAnalysis::single_segs,
                        singleSegs), /**< Line segments which could not be clustered to a line */
  FIELD_WRAPPER_DEFAULT(std::vector<LineSegment>,
                        nomadz_msgs::msg::LineAnalysis::raw_segs,
                        rawSegs), /**< All line segments before clustering */
  FIELD_WRAPPER_DEFAULT(CircleSpot,
                        nomadz_msgs::msg::LineAnalysis::circle,
                        circle), /**< The position of the center circle if found */
});
