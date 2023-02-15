/**
 * @file BodyContour.h
 *
 * The file declares a class that represents the contour of the robot's body in the image.
 * The contour can be used to exclude the robot's body from image processing.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Core/Math/Vector2.h"
#include "Core/Streams/AutoStreamable.h"

/**
 * @class BodyContour
 * A class that represents the contour of the robot's body in the image.
 * The contour can be used to exclude the robot's body from image processing.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/body_contour_line.hpp"
#endif
STREAMABLE_DECLARE(BodyContourLine)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/body_contour.hpp"
#endif
STREAMABLE_DECLARE(BodyContour)

STREAMABLE_ROS(BodyContour, {
public:
  /** A class representing a line in 2-D space. */
  STREAMABLE_ROS(BodyContourLine, {
  public:
    /**
     * Constructor.
     * @param p1 The first endpoint of the line.
     * @param p2 The second endpoint of the line.
     */
    BodyContourLine(const Vector2<int>& p1, const Vector2<int>& p2);

    /**
     * The method determines the x coordinate of the line for a certain y coordinate
     * if the is any.
     * @param y The y coordinate.
     * @param x The x coordinate is returned here if it exists.
     * @return Does such an x coordinate exist?
     */
    bool xAt(int y, int& x) const {
      if ((p1.y <= y && p2.y > y) || (p1.y > y && p2.y <= y)) {
        x = p1.x + (p2.x - p1.x) * (y - p1.y) / (p2.y - p1.y);
        return true;
      } else
        return false;
    }

    /**
     * The method determines the y coordinate of the line for a certain x coordinate
     * if the is any.
     * @param x The x coordinate.
     * @param y The y coordinate is returned here if it exists.
     * @return Does such a y coordinate exist?
     */
    bool yAt(int x, int& y) const {
      if (p1.x <= x && p2.x > x) {
        y = p1.y + (p2.y - p1.y) * (x - p1.x) / (p2.x - p1.x);
        return true;
      } else
        return false;
    }
    , FIELD_WRAPPER_DEFAULT(Vector2<int>, nomadz_msgs::msg::BodyContourLine::p1, p1), /**< The left point of the line. */
      FIELD_WRAPPER_DEFAULT(Vector2<int>, nomadz_msgs::msg::BodyContourLine::p2, p2), /**< The right point of the line. */
  });

  using Line = BodyContourLine;

  /**
   * The method clips the bottom y coordinate of a vertical line.
   * @param x The x coordinate of the vertical line.
   * @param y The original y coordinate of the bottom of the vertical line.
   *          It will be replaced if necessary. Note that the resulting point
   *          can be outside the image!
   */
  void clipBottom(int x, int& y) const;

  /**
   * The method clips the bottom y coordinate of a vertical line.
   * @param x The x coordinate of the vertical line.
   * @param y The original y coordinate of the bottom of the vertical line.
   *          It will be replaced with a point inside the image.
   * @param imageHeight Is used to determine if the clipped y coordinate is outside
   *        the image.
   */
  void clipBottom(int x, int& y, int imageHeight) const;

  /**
   * The method clips the left x coordinate of a horizonal line.
   * It only consides descending clipping lines.
   * @param x The original x coordinate of the left end of the horizontal line.
   *          It will be replaced if necessary. Note that the resulting point
   *          can be outside the image!
   * @param y The y coordinate of the horizontal line.
   */
  void clipLeft(int& x, int y) const;

  /**
   * The method clips the right x coordinate of a horizonal line.
   * It only consides ascending clipping lines.
   * @param x The original x coordinate of the right end of the horizontal line.
   *          It will be replaced if necessary. Note that the resulting point
   *          can be outside the image!
   * @param y The y coordinate of the horizontal line.
   */
  void clipRight(int& x, int y) const;

  /**
   * Returns the y coordinate of the highest visible point.
   * @return
   */
  int getMaxY() const;

  /** Creates drawings of the contour. */
  void draw() const,
    FIELD_WRAPPER_DEFAULT(
      std::vector<BodyContourLine>, nomadz_msgs::msg::BodyContour::lines, lines), /**< The clipping lines. */
    FIELD_WRAPPER_DEFAULT(
      Vector2<int>, nomadz_msgs::msg::BodyContour::camera_resolution, cameraResolution), /**< Only for drawing. */

    // Initialization
    lines.reserve(50);
});
