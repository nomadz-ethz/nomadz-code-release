/**
 * @file Shapes.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Streams/FieldWrapper.h"
#include "Core/Math/Vector2.h"
#include "Core/Math/Vector3.h"
#include "Core/Math/Pose2D.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/line.hpp"
#endif
STREAMABLE_DECLARE(Line)

namespace Geometry {
  /** Defines a circle by its center and its radius*/
  struct Circle {
    Circle() : radius(0) {}
    Circle(const Vector2<>& c, float r) {
      center = c;
      radius = r;
    }
    Vector2<> center;
    float radius;
  };

  // clang-format off
  /** Defines a line by two vectors*/
  STREAMABLE_ROS(Line, {
    public:
    Line(const Vector2<>& base, const Vector2<>& direction);

    static Line fromPoints(const Vector2<>& p1, const Vector2<>& p2);
    
    float length() const { return direction.abs(); }

    void normalizeDirection(), 

    FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::Line::base, base),
    FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::Line::direction, direction),
  });
  // clang-format on

  struct PixeledLine {
    PixeledLine(int x1, int x2, int y1, int y2) : x1(x1), y1(y1), x2(x2), y2(y2) { calculatePixels(); };

    PixeledLine(const Vector2<int>& start, const Vector2<int>& end) : x1(start.x), y1(start.y), x2(end.x), y2(end.y) {
      calculatePixels();
    };

    void calculatePixels() {
      char sign;
      if (x1 == x2 && y1 == y2) {
        numberOfPixels = 0;
      } else // begin and end differ
      {
        if (abs(x2 - x1) > abs(y2 - y1)) {
          if (x1 < x2)
            sign = 1;
          else
            sign = -1;
          numberOfPixels = abs(x2 - x1) + 1;
          for (int x = 0; x < numberOfPixels; x++) {
            int y = (int)(x * (y2 - y1) / (x2 - x1));
            x_coordinate[x] = x1 + x * sign;
            y_coordinate[x] = y1 + y * sign;
          }
        } else {
          if (y1 < y2)
            sign = 1;
          else
            sign = -1;
          numberOfPixels = abs(y2 - y1) + 1;
          for (int y = 0; y < numberOfPixels; y++) {
            int x = (int)(y * (x2 - x1) / (y2 - y1));
            x_coordinate[y] = x1 + x * sign;
            y_coordinate[y] = y1 + y * sign;
          }
        }
      } // begin and end differ
    }   // calculatePixels

    inline int getNumberOfPixels() const { return numberOfPixels; }

    inline int getPixelX(int i) const { return x_coordinate[i]; }

    inline int getPixelY(int i) const { return y_coordinate[i]; }

  private:
    int x1, y1, x2, y2;
    int numberOfPixels;
    enum { maxNumberOfPixelsInLine = 600 }; // diagonal size of BW image
    int x_coordinate[maxNumberOfPixelsInLine];
    int y_coordinate[maxNumberOfPixelsInLine];
  };

  /**
   * Returns the circle defined by the three points.
   * @param point1 The first point.
   * @param point2 The second point.
   * @param point3 The third point.
   * @return The circle defined by point1, point2 and point3.
   */
  Circle getCircle(const Vector2<int>& point1, const Vector2<int>& point2, const Vector2<int>& point3);

  int getIntersectionOfCircles(const Circle& c1, const Circle& c2, Vector2<>& p1, Vector2<>& p2);

  /**
   * Computes the intersection point of a line and a circle.
   * @param line The Line.
   * @param circle The Circle.
   * @param firstIntersection The first intersection point, if there is one.
   * @param secondIntersection The second intersection point, if there is one.
   * @return The number of intersection points.
   */
  int getIntersectionOfLineAndCircle(const Line& line,
                                     const Circle& circle,
                                     Vector2<>& firstIntersection,
                                     Vector2<>& secondIntersection);

  /**
   * Keep part of line segment inside of circle
   * TODO Should work on circles not centered at (0.f, 0.f), but needs more testing?
   * @param circle The circle.
   * @param p1 Endpoint 1 of the line segment (can be modified)
   * @param p2 Endpoint 2 of the line segment (can be modified)
   * @return Whether (if false, p1 & p2 are unmodified)
   */
  bool clipLineWithCircle(const Circle& circle, Vector2<>& p1, Vector2<>& p2);

  // Returns the two points on the circle that touch the tangent lines going through p
  // Returns an empty vector if p is on or inside the circle
  std::vector<Vector2<>> getTangentsToPoint(const Circle& c, const Vector2<>& p);

  bool checkIntersectionOfLines(const Vector2<>& l1p1, const Vector2<>& l1p2, const Vector2<>& l2p1, const Vector2<>& l2p2);

  bool getIntersectionOfLines(const Line& line1, const Line& line2, Vector2<>& intersection);

  bool getIntersectionOfLines(const Line& line1, const Line& line2, Vector2<int>& intersection);

  bool getIntersectionOfRaysFactor(const Line& ray1, const Line& ray2, float& intersection);

  float getDistanceToLine(const Line& line, const Vector2<>& point);

  float getDistanceToEdge(const Line& line, const Vector2<>& point);

  float distance(const Vector2<>& point1, const Vector2<>& point2);

  float distance(const Vector2<int>& point1, const Vector2<int>& point2);

  bool clipLineWithQuadrangle(const Line& lineToClip,
                              const Vector2<>& corner0,
                              const Vector2<>& corner1,
                              const Vector2<>& corner2,
                              const Vector2<>& corner3,
                              Vector2<>& clipPoint1,
                              Vector2<>& clipPoint2);

  bool clipLineWithQuadrangle(const Line& lineToClip,
                              const Vector2<>& corner0,
                              const Vector2<>& corner1,
                              const Vector2<>& corner2,
                              const Vector2<>& corner3,
                              Vector2<int>& clipPoint1,
                              Vector2<int>& clipPoint2);

  bool isPointInsideRectangle(const Vector2<>& bottomLeftCorner, const Vector2<>& topRightCorner, const Vector2<>& point);

  bool isPointInsideRectangle2(const Vector2<>& corner1, const Vector2<>& corner2, const Vector2<>& point);

  bool isPointInsideRectangle(const Vector2<int>& bottomLeftCorner,
                              const Vector2<int>& topRightCorner,
                              const Vector2<int>& point);

  bool isPointInsideConvexPolygon(const Vector2<> polygon[], const int numberOfPoints, const Vector2<>& point);

  /**
   * Clips a line with a rectangle
   * @param bottomLeft The bottom left corner of the rectangle
   * @param topRight The top right corner of the rectangle
   * @param line The line to be clipped
   * @param point1 The starting point of the resulting line
   * @param point2 The end point of the resulting line
   * @return states whether clipping was necessary (and done)
   */
  bool getIntersectionPointsOfLineAndRectangle(const Vector2<int>& bottomLeft,
                                               const Vector2<int>& topRight,
                                               const Line line,
                                               Vector2<int>& point1,
                                               Vector2<int>& point2);

  bool getIntersectionPointsOfLineAndRectangle(
    const Vector2<>& bottomLeft, const Vector2<>& topRight, const Line line, Vector2<>& point1, Vector2<>& point2);

  /**
   * Calculate the segment of a line inside a rectangle
   * @param topLeft The (low x, low y) corner of the rectangle
   * @param bottomRight The (high x, high y) corner of the rectangle
   * @param point1 Endpoint 1 of the line segment (can be modified)
   * @param point2 Endpoint 2 of the line segment (can be modified)
   * @return states whether line exists after clipping
   * @see http://de.wikipedia.org/wiki/Algorithmus_von_Cohen-Sutherland
   */
  bool clipLineWithRectangle(const Vector2<int>& topLeft,
                             const Vector2<int>& bottomRight,
                             Vector2<int>& point1,
                             Vector2<int>& point2);
} // namespace Geometry