/**
 * @file FieldDimensions.h
 *
 * Description of the dimensions of the field.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are Matthias Jüngel, Max Risler and Thomas Röfer.
 */

#pragma once

#include "Core/Range.h"
#include "Core/Math/Pose2D.h"
#include "Core/Enum.h"
#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/circle.hpp"
#endif
STREAMABLE_DECLARE(Circle)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/field_dimensions.hpp"
#endif
STREAMABLE_DECLARE(FieldDimensions)

/**
 * A stremable representing a field-line segment.
 *
 */
struct FieldDescriptionLine {
  FieldDescriptionLine() = default;
  FieldDescriptionLine(Vector2<> from, Vector2<> to, bool isPartOfCircle = false);

  Pose2D corner() const;
  void corner(const Pose2D& corner);
  float rotation() const;
  float length() const;

  Vector2<> from;
  Vector2<> to;
  bool isPartOfCircle = false;
};

STREAMABLE_ROS(
  Circle,
  {
    ,
    FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::Circle::center, center), /**< The center of the circle. */
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::Circle::radius, radius),        /**< The radius of the circle. */
    FIELD_WRAPPER(int,
                  0,
                  nomadz_msgs::msg::Circle::num_of_segments,
                  numOfSegments), /**< The number of segments used to discretize the circle. */
  });

/**
 * This is a collection of line- or boundary segments with start-Pose2D and length.
 */
class LinesTable {
public:
  std::vector<FieldDescriptionLine> lines;

  void push(const Vector2<>& s, const Vector2<>& e, bool isPartOfCircle = false);

  void pushCircle(const Vector2<>& center, float radius, int numOfSegments);

  /*
   * Returns whether a given point is inside the polygon described by the line segments.
   * Only valid if the line segment table describes a closed polygon.
   */
  bool isInside(const Vector2<>& v) const;

  /**
   * The function clips a point to the polygon described by the line segments.
   * Only valid if the line segment table describes a closed polygon.
   * @param v The point.
   * @return How far was the point moved?
   */
  float clip(Vector2<>& v) const;

  /**
   * The function returns the point on a line of a certain type closest to given a point.
   * @param point The point on a line.
   * @param p The reference point and the rotation of the line.
   * @param numberOfRotations The number of discretizations of line rotations.
   * @param minLength The minimum length of the line segments that are considered.
   * @return whether there is a matching point in that direction
   */
  bool getClosestPoint(Vector2<>& point, const Pose2D& p, int numberOfRotations, float minLength) const;

  /**
   * The function returns the distance between a point and the closest point on a line of a certain type in a certain
   * direction.
   * @param pose The reference point and direction.
   * @return The distance. It is -1 if no line of that type exists in the certain direction.
   */
  float getDistance(const Pose2D& pose) const;
};

/**
 * The class represents all corners of a certain type.
 */
class CornersTable : public std::vector<Vector2<>> {
public:
  /**
   * The method returns the position of the corner closest to a point.
   * The method is only defined if !empty().
   * @param p The point.
   * @return The position of the closest corner.
   */
  const Vector2<>& getClosest(const Vector2<>& p) const;

  /**
   * The method returns the position of the corner closest to a point.
   * The method is only defined if !empty().
   * @param p The point.
   * @return The position of the closest corner.
   */
  Vector2<int> getClosest(const Vector2<int>& p) const;
};

/**
 * Class containing definitions and functions
 * regarding field dimensions.
 */
STREAMABLE_ROS(FieldDimensions, {
public:
  /**
   * @brief Get all the field lines, including the center circle segments
   *
   * @return std::vector<FieldDescriptionLine>
   */
  std::vector<FieldDescriptionLine> getAllFieldLines() const;

  /**
   * @brief Get the field lines, without the center circle segments
   *
   * @return std::vector<FieldDescriptionLine>
   */
  std::vector<FieldDescriptionLine> getFieldLinesWithoutCenterCircle() const;

  /**
   * @brief Get only the lines belonging to the field border
   *
   * @return std::vector<FieldDescriptionLine>
   */
  std::vector<FieldDescriptionLine> getFieldBorderLines() const;

  /**
   * @brief Get borders of the carpet as field lines
   *
   * @return std::vector<FieldDescriptionLine>
   */
  std::vector<FieldDescriptionLine> getCarpetBorderLines() const;

  /**
   * Read field dimensions from configuration file.
   */
  void load();

  /**
   * Returns true when p is inside the carpet.
   */
  bool isInsideCarpet(const Vector2<>& p) const;

  /**
   * The function clips a point to the carpet.
   * @param v The point.
   * @return How far was the point moved?
   */
  float clipToCarpet(Vector2<> & v) const;

  /**
   * Returns true when p is inside the playing field.
   */
  bool isInsideField(const Vector2<>& p) const;

  /**
   * The function returns a random pose inside the field.
   * @return The random pose.
   */
  Pose2D randomPoseOnField() const;

  /**
   * The function returns a random pose on the carpet.
   * @return The random pose.
   */
  Pose2D randomPoseOnCarpet() const;

  /**
   * The method draws the field lines.
   */
  void draw() const;

  /**
   * The method draws the field polygons.
   * @param ownColor The color of the own team.
   */
  void drawPolygons(unsigned ownColor) const;

private:
  /**
   * Collection of lines describing a polygon around the border of the field carpet.
   * All legal robot positions are inside this polygon.
   */
  mutable std::vector<FieldDescriptionLine> carpetBorderLines;
  /**
   * Collection of lines describing a polygon around the border of the playing field.
   * All legal ball positions are inside this polygon.
   */
  mutable std::vector<FieldDescriptionLine> fieldBorderLines;

  /**
   * All the line segments that make up the field.
   */
  mutable std::vector<FieldDescriptionLine> fieldLines;

  // FIXME(albanesg): the stuff below doesn't compile inside an autostreamable
  // because of the enum class - need to find a workaround though to be fair
  // it is not really needed

  // /**
  //  * All different corner classes.
  //  */
  // enum class CornerClass {
  //   xCorner,
  //   tCorner0,
  //   tCorner90,
  //   tCorner180,
  //   tCorner270,
  //   lCorner0,
  //   lCorner90,
  //   lCorner180,
  //   lCorner270
  // };

  // std::vector<Vector2<>> getCornersOfClass(CornerClass cornerClass) const;
  /**
   * The method draws the field lines.
   */
  void drawLines() const;

  /**
   * The method draws the field lines.
   */
  void drawCorners() const;

  /**
   * The function clips a point to the field.
   * @param v The point.
   * @return How far was the point moved?
   */
  float clipToField(Vector2<> & v) const;

  static bool isInside(const std::vector<FieldDescriptionLine>& lines, const Vector2<>& v);

  static float clip(const std::vector<FieldDescriptionLine>& lines, Vector2<>& v);

  std::vector<FieldDescriptionLine> getCenterCircleLineSegments() const,

    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_opponent_field_border, xPosOpponentFieldBorder),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_opponent_goal, xPosOpponentGoal),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_opponent_goal_post, xPosOpponentGoalPost),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_opponent_groundline, xPosOpponentGroundline),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_opponent_penalty_area, xPosOpponentPenaltyArea),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_opponent_goal_box, xPosOpponentGoalBox),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_opponent_drop_in_line, xPosOpponentDropInLine),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_opponent_penalty_mark, xPosOpponentPenaltyMark),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_half_way_line, xPosHalfWayLine),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_own_penalty_area, xPosOwnPenaltyArea),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_own_goal_box, xPosOwnGoalBox),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_own_drop_in_line, xPosOwnDropInLine),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_own_penalty_mark, xPosOwnPenaltyMark),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_own_groundline, xPosOwnGroundline),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_own_goal_post, xPosOwnGoalPost),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_own_goal, xPosOwnGoal),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::x_pos_own_field_border, xPosOwnFieldBorder),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::y_pos_left_field_border, yPosLeftFieldBorder),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::y_pos_left_sideline, yPosLeftSideline),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::y_pos_left_drop_in_line, yPosLeftDropInLine),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::y_pos_left_penalty_area, yPosLeftPenaltyArea),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::y_pos_left_goal_box, yPosLeftGoalBox),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::y_pos_left_goal, yPosLeftGoal),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::y_pos_center_goal, yPosCenterGoal),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::y_pos_right_goal, yPosRightGoal),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::y_pos_right_penalty_area, yPosRightPenaltyArea),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::y_pos_right_goal_box, yPosRightGoalBox),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::y_pos_right_drop_in_line, yPosRightDropInLine),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::y_pos_right_sideline, yPosRightSideline),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::y_pos_right_field_border, yPosRightFieldBorder),

    // other dimensions

    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::field_lines_width, fieldLinesWidth),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::center_circle_radius, centerCircleRadius),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::goal_post_radius, goalPostRadius),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::goal_height, goalHeight),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::ball_radius, ballRadius),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::FieldDimensions::ball_friction, ballFriction), // in 1/s

    /**
     * Stores the center circle (serialized in from a file, or received from another process)
     */
    FIELD_WRAPPER_DEFAULT(Circle, nomadz_msgs::msg::FieldDimensions::center_circle, centerCircle),

  // FIELD_WRAPPER_DEFAULT(std::vector<Vector2<>>, nomadz_msgs::msg::FieldDimensions::x_corner, xCorner),
  // FIELD_WRAPPER_DEFAULT(std::vector<Vector2<>>, nomadz_msgs::msg::FieldDimensions::t_corner_0, tCorner0),
  // FIELD_WRAPPER_DEFAULT(std::vector<Vector2<>>, nomadz_msgs::msg::FieldDimensions::t_corner_90, tCorner90),
  // FIELD_WRAPPER_DEFAULT(std::vector<Vector2<>>, nomadz_msgs::msg::FieldDimensions::t_corner_180, tCorner180),
  // FIELD_WRAPPER_DEFAULT(std::vector<Vector2<>>, nomadz_msgs::msg::FieldDimensions::t_corner_270, tCorner270),
  // FIELD_WRAPPER_DEFAULT(std::vector<Vector2<>>, nomadz_msgs::msg::FieldDimensions::l_corner_0, lCorner0),
  // FIELD_WRAPPER_DEFAULT(std::vector<Vector2<>>, nomadz_msgs::msg::FieldDimensions::l_corner_90, lCorner90),
  // FIELD_WRAPPER_DEFAULT(std::vector<Vector2<>>, nomadz_msgs::msg::FieldDimensions::l_corner_180, lCorner180),
  // FIELD_WRAPPER_DEFAULT(std::vector<Vector2<>>, nomadz_msgs::msg::FieldDimensions::l_corner_270, lCorner270),
});
