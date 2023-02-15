/**
 * @file LineLocalization.h
 *
 * Pose hypotheses (and their uncertainty) from line-based localization
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Debugging/DebugDrawings.h"
#include "Core/Math/Pose2D.h"
#include "Core/Math/Vector2.h"
#include "Core/Streams/AutoStreamable.h"
#include "Core/Streams/FieldWrapper.h"
#include "Representations/Perception/LineAnalysis.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/localization_line.hpp"
#endif
STREAMABLE_DECLARE(LocalizationLine)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/localization_circle_spot.hpp"
#endif
STREAMABLE_DECLARE(LocalizationCircleSpot)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/localization_intersection.hpp"
#endif
STREAMABLE_DECLARE(LocalizationIntersection)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/localization_penalty_area_side.hpp"
#endif
STREAMABLE_DECLARE(LocalizationPenaltyAreaSide)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/hypothesis.hpp"
#endif
STREAMABLE_DECLARE(Hypothesis)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/line_localization.hpp"
#endif
STREAMABLE_DECLARE(LineLocalization)

STREAMABLE_ROS(LineLocalization, {
public:
  /**
   * @class Line
   *
   * This class represents a found fieldline.
   */
STREAMABLE_ROS(LocalizationLine, {
  public:
    /**
     * Constructor for importing from LineAnalyses
     * @param line a field line from LineAnalysis
     */
        LocalizationLine (const LineAnalysis::ObservedFieldLine& line): LocalizationLine() {
      alpha = line.alpha; d = line.d; dead = line.dead; midLine = line.midLine;
      first = line.first; last = line.last;
      startInImage = line.startInImage; endInImage = line.endInImage;
    }

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
    Vector2<> calculateClosestPointOnLine(const Vector2<>& p) const
    {
  // const Vector2<> x0 = p - first;
  // const Vector2<> v = Vector2<>(last.x - first.x, last.y - first.y);
  // const Vector2<> proj = v * (x0.x * v.x + x0.y * v.y) / v.squareAbs();
  // OUTPUT(idText, text, "first = " << first.x << ", " << first.y << "; last = " << last.x << ", " << last.y << "; p = " <<
  // p.x << ", " << p.y << "; x0 = " << x0.x << ", " << x0.y << "; v = " << v.x << ", " << v.y << "; proj = " << proj.x << ",
  // " << proj.y << "; return " << (first+proj).x << ", " << (first+proj).y);
  // return first + proj;

  const Vector2<> normale = Vector2<>(std::cos(alpha + (float)M_PI), std::sin(alpha + (float)M_PI));
  return p + normale * calculateDistToLine(p);
    }

    /**
     * Returns true if closest point of `p` to line is between line.first & line.last
     * @param p a point
     * @return true if it fits
     */
    bool pointIsBesideSegment(const Vector2<>& p) const
    {
  const Vector2<> pointOnLine = calculateClosestPointOnLine(p);
  const Vector2<> midpoint = (first + last) * 2.f;
  const float len = (first - last).abs();
  return ((pointOnLine - midpoint).abs() < len);
    },
FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::LocalizationLine::alpha, alpha), /**< direction of this line in Hesse norm form */
FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::LocalizationLine::d, d), /**< distance of this line in Hesse norm form */
FIELD_WRAPPER_DEFAULT(bool, nomadz_msgs::msg::LocalizationLine::dead, dead), /**< This is needed for merging lines */
FIELD_WRAPPER_DEFAULT(bool, nomadz_msgs::msg::LocalizationLine::mid_line, midLine), /**< Whether this is the line throught the center circle */
FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::LocalizationLine::first, first), /**< The starting point of this line in field coordinates */
FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::LocalizationLine::last, last), /**< The end point of this line in field coordinates */
FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::LocalizationLine::start_in_image, startInImage), /**< The start point of this line in image coordinates */
FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::LocalizationLine::end_in_image, endInImage), /**< The end point of this line in image coordinates */
});

using Line = LocalizationLine;

/**
 * @class CircleSpot
 *
 * This class represents circle spots. A circle spot
 * is a point calculated from a linesegment where the
 * center circle would be if the linesegment is part
 * of the center circle.
 * This is also used for the found circle.
 */
STREAMABLE_ROS(LocalizationCircleSpot, {
  public:
    /**
     * Constructor for importing from LineAnalyses
     * @param circleSpot a circle from LineAnalysis
     */
        LocalizationCircleSpot (const LineAnalysis::CircleSpot& circleSpot): LocalizationCircleSpot() {
      pos = circleSpot.pos; found = circleSpot.found; lastSeen = circleSpot.lastSeen;
}
,
  FIELD_WRAPPER_DEFAULT(Vector2<float>,
                        nomadz_msgs::msg::LocalizationCircleSpot::pos,
                        pos), /**< The position of the center of the center circle in field coordinates */
  FIELD_WRAPPER(bool,
                false,
                nomadz_msgs::msg::LocalizationCircleSpot::found,
                found), /**< Whether the center circle was found in this frame */
  FIELD_WRAPPER(unsigned,
                0,
                nomadz_msgs::msg::LocalizationCircleSpot::last_seen,
                lastSeen), /**< The last time the center circle was seen */
});

using CircleSpot = LocalizationCircleSpot;

/**
 * @class Intersection
 * A class representing a intersection of two fieldlines
 */
STREAMABLE_ROS(LocalizationIntersection, {
  public:
    /**
     * Constructor for importing from LineAnalyses
     * @param intersection an intersection from LineAnalysis
     */
        LocalizationIntersection (const LineAnalysis::Intersection& intersection): LocalizationIntersection() {
      type = (IntersectionType)((int)intersection.type); pos = intersection.pos;
      dir1 = intersection.dir1; dir2 = intersection.dir2;
      first1 = intersection.first1; last1 = intersection.last1;
      first2 = intersection.first2; last2 = intersection.last2;
}

ENUM(IntersectionType, L, T, X)
, FIELD_WRAPPER_DEFAULT(IntersectionType, nomadz_msgs::msg::LocalizationIntersection::type, type),
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::LocalizationIntersection::pos,
                        pos), /**< The fieldcoordinates of the intersection */
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::LocalizationIntersection::dir1,
                        dir1), /**< The first direction of the lines intersected. */
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::LocalizationIntersection::dir2,
                        dir2), /**< The second direction of the lines intersected. */
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::LocalizationIntersection::first1,
                        first1), /**< Starting point of line crossing this intersection in direction dir1 */
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::LocalizationIntersection::last1,
                        last1), /**< Ending point of line crossing this intersection in direction dir1 */
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::LocalizationIntersection::first2,
                        first2), /**< Starting point of line crossing this intersection in direction dir2 */
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::LocalizationIntersection::last2,
                        last2), /**< Ending point of line crossing this intersection in direction dir2 */
});

using Intersection = LocalizationIntersection;

/**
 * @class PenaltyAreaSide
 * A class representing a side of the penalty area, also a T-intersection connected to an L-intersection.
 */
STREAMABLE_ROS(LocalizationPenaltyAreaSide, {
  public :,
  FIELD_WRAPPER_DEFAULT(LocalizationIntersection,
                        nomadz_msgs::msg::LocalizationPenaltyAreaSide::corner,
                        corner), /**< The L corner on this side */
  FIELD_WRAPPER_DEFAULT(LocalizationIntersection,
                        nomadz_msgs::msg::LocalizationPenaltyAreaSide::tee,
                        tee), /**< The T intersection on this side */
  FIELD_WRAPPER_DEFAULT(bool,
                        nomadz_msgs::msg::LocalizationPenaltyAreaSide::is_left,
                        isLeft), /**< True if left side of penalty area */
});

using PenaltyAreaSide = LocalizationPenaltyAreaSide;

/**
 * @class Hypothesis
 * A tuple with pose, uncertainty, and color.
 */
STREAMABLE_ROS(Hypothesis, {
  public:
      Hypothesis(const Pose2D pose, const float dist, const ColorRGBA color): Hypothesis() {
      this->pose = pose;
      this->dist = dist;
      this->color[0] = color.r;
      this->color[1] = color.g;
      this->color[2] = color.b;
      this->color[3] = color.a;
}
, FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::Hypothesis::pose, pose),
  FIELD_WRAPPER_DEFAULT(float,
                        nomadz_msgs::msg::Hypothesis::dist,
                        dist), /**< distance of field feature (mm); proportional to covariance/uncertainty */
  FIELD_WRAPPER_DEFAULT(unsigned char[4], nomadz_msgs::msg::Hypothesis::color, color),
}),
FIELD_WRAPPER_DEFAULT(std::vector<Hypothesis>, nomadz_msgs::msg::LineLocalization::hypotheses, hypotheses),
});
