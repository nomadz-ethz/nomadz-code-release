/**
 * @file Shapes.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#include "Shapes.h"

namespace {
  int ccw(const Vector2<>& p0, const Vector2<>& p1, const Vector2<>& p2) {
    float dx1(p1.x - p0.x);
    float dy1(p1.y - p0.y);
    float dx2(p2.x - p0.x);
    float dy2(p2.y - p0.y);
    if (dx1 * dy2 > dy1 * dx2) {
      return 1;
    }
    if (dx1 * dy2 < dy1 * dx2) {
      return -1;
    }
    // Now (dx1*dy2 == dy1*dx2) must be true:
    if ((dx1 * dx2 < 0.0f) || (dy1 * dy2 < 0.0f)) {
      return -1;
    }
    if ((dx1 * dx1 + dy1 * dy1) >= (dx2 * dx2 + dy2 * dy2)) {
      return 0;
    }
    return 1;
  }
} // namespace

Geometry::Line::Line(const Vector2<>& base, const Vector2<>& direction) : base(base), direction(direction) {}

Geometry::Line Geometry::Line::fromPoints(const Vector2<>& p1, const Vector2<>& p2) {
  return Line(p1, p2 - p1);
}

void Geometry::Line::normalizeDirection() {
  float distance = sqrt(sqr(direction.x) + sqr(direction.y));
  direction.x = direction.x / distance;
  direction.y = direction.y / distance;
}

Geometry::Circle Geometry::getCircle(const Vector2<int>& point1, const Vector2<int>& point2, const Vector2<int>& point3) {
  float x1 = (float)point1.x;
  float y1 = (float)point1.y;
  float x2 = (float)point2.x;
  float y2 = (float)point2.y;
  float x3 = (float)point3.x;
  float y3 = (float)point3.y;

  Circle circle;

  const float temp = x2 * y1 - x3 * y1 - x1 * y2 + x3 * y2 + x1 * y3 - x2 * y3;

  if (temp == 0) {
    circle.radius = 0;
  } else {
    circle.radius =
      0.5f *
      sqrt(((sqr(x1 - x2) + sqr(y1 - y2)) * (sqr(x1 - x3) + sqr(y1 - y3)) * (sqr(x2 - x3) + sqr(y2 - y3))) / sqr(temp));
  }

  if (temp == 0) {
    circle.center.x = 0;
  } else {
    circle.center.x =
      (sqr(x3) * (y1 - y2) + (sqr(x1) + (y1 - y2) * (y1 - y3)) * (y2 - y3) + sqr(x2) * (-y1 + y3)) / (-2.0f * temp);
  }

  if (temp == 0) {
    circle.center.y = 0;
  } else {
    circle.center.y = (sqr(x1) * (x2 - x3) + sqr(x2) * x3 + x3 * (-sqr(y1) + sqr(y2)) - x2 * (+sqr(x3) - sqr(y1) + sqr(y3)) +
                       x1 * (-sqr(x2) + sqr(x3) - sqr(y2) + sqr(y3))) /
                      (2.0f * temp);
  }
  return circle;
}

int Geometry::getIntersectionOfCircles(const Circle& c0, const Circle& c1, Vector2<>& p1, Vector2<>& p2) {
  float a, dx, dy, d, h, rx, ry;
  float x2, y2;

  /* dx and dy are the vertical and horizontal distances between
   * the circle centers.
   */
  dx = c1.center.x - c0.center.x;
  dy = c1.center.y - c0.center.y;

  /* Determine the straight-line distance between the centers. */
  d = sqrt((dy * dy) + (dx * dx));

  /* Check for solvability. */
  if (d > (c0.radius + c1.radius)) {
    /* no solution. circles do not intersect. */
    return 0;
  }
  if (d < std::abs(c0.radius - c1.radius)) {
    /* no solution. one circle is contained in the other */
    return 0;
  }

  /* 'point 2' is the point where the line through the circle
   * intersection points crosses the line between the circle
   * centers.
   */

  /* Determine the distance from point 0 to point 2. */
  a = ((c0.radius * c0.radius) - (c1.radius * c1.radius) + (d * d)) / (2.0f * d);

  /* Determine the coordinates of point 2. */
  x2 = c0.center.x + (dx * a / d);
  y2 = c0.center.y + (dy * a / d);

  /* Determine the distance from point 2 to either of the
   * intersection points.
   */
  h = sqrt((c0.radius * c0.radius) - (a * a));

  /* Now determine the offsets of the intersection points from
   * point 2.
   */
  rx = -dy * (h / d);
  ry = dx * (h / d);

  /* Determine the absolute intersection points. */
  p1.x = x2 + rx;
  p2.x = x2 - rx;
  p1.y = y2 + ry;
  p2.y = y2 - ry;

  return 1;
}

bool Geometry::getIntersectionOfLines(const Line& line1, const Line& line2, Vector2<int>& intersection) {
  Vector2<> intersectionDouble;
  bool toReturn = getIntersectionOfLines(line1, line2, intersectionDouble);
  intersection.x = (int)intersectionDouble.x;
  intersection.y = (int)intersectionDouble.y;
  return toReturn;
}

bool Geometry::getIntersectionOfLines(const Line& line1, const Line& line2, Vector2<>& intersection) {
  if (line1.direction.y * line2.direction.x == line1.direction.x * line2.direction.y) {
    return false;
  }

  intersection.x = line1.base.x + line1.direction.x *
                                    (line1.base.y * line2.direction.x - line2.base.y * line2.direction.x +
                                     (-line1.base.x + line2.base.x) * line2.direction.y) /
                                    ((-line1.direction.y) * line2.direction.x + line1.direction.x * line2.direction.y);

  intersection.y = line1.base.y + line1.direction.y *
                                    (-line1.base.y * line2.direction.x + line2.base.y * line2.direction.x +
                                     (line1.base.x - line2.base.x) * line2.direction.y) /
                                    (line1.direction.y * line2.direction.x - line1.direction.x * line2.direction.y);

  return true;
}

int Geometry::getIntersectionOfLineAndCircle(const Line& line,
                                             const Circle& circle,
                                             Vector2<>& firstIntersection,
                                             Vector2<>& secondIntersection) {
  /* solves the following system of equations:
   *
   * (x - x_m)^2 + (y - y_m)^2 = r^2
   * p + l * v = [x, y]
   *
   * where [x_m, y_m] is the center of the circle,
   * p is line.base and v is line.direction and
   * [x, y] is an intersection point.
   * Solution was found with the help of maple.
   */
  const float divisor = line.direction.squareAbs();
  const float p = 2 * (line.base * line.direction - circle.center * line.direction) / divisor;
  const float q = ((line.base - circle.center).sqr() - sqr(circle.radius)) / divisor;
  const float p_2 = p / 2.0f;
  const float radicand = sqr(p_2) - q;
  if (radicand < 0) {
    return 0;
  } else {
    const float radix = sqrt(radicand);
    firstIntersection = line.base + line.direction * (-p_2 + radix);
    secondIntersection = line.base + line.direction * (-p_2 - radix);
    return radicand == 0 ? 1 : 2;
  }
}

bool Geometry::clipLineWithCircle(const Geometry::Circle& circle, Vector2<>& p1, Vector2<>& p2) {
  const Vector2<>& c = circle.center;
  const float r2 = circle.radius * circle.radius;

  // Work inside translated frame with origin at circle.center
  const Vector2<> rel1 = p1 - circle.center;
  const Vector2<> rel2 = p2 - circle.center;

  Vector2<> inter1, inter2;
  const Line line = {rel1, rel2 - rel1};
  const int intersections = getIntersectionOfLineAndCircle(line, circle, inter1, inter2);

  // Completely outside of the circle
  if (intersections != 2) {
    return false;
  }

  if (rel1.squareAbs() >= r2 && rel2.squareAbs() >= r2) {
    // Line cuts through circle

    // Keep only if inter1, inter2 are between rel1 & rel2
    // (p1 & p2 are outside of the circle, so only checking one of the inter is necessary)
    if (!isPointInsideRectangle2(rel1, rel2, inter1)) {
      return false;
    }

    p1 = inter1 + c;
    p2 = inter2 + c;

  } else {
    // Line segment starts inside circle (and might end anywhere, we don't know)

    // Find which is nearer to/farther from center circle
    Vector2<> near = rel1, far = rel2;
    if (near.squareAbs() > far.squareAbs()) {
      std::swap(near, far);
    }

    // If "far" is outside range, "drag" it "back"
    if (far.squareAbs() >= r2) {
      // Set far to whichever of (inter1, inter2) is between near & far
      far = isPointInsideRectangle2(near, far, inter1) ? inter1 : inter2;
    }

    // Translate back to original frame
    p1 = near + c;
    p2 = far + c;
  }

  return true;
}

std::vector<Vector2<>> Geometry::getTangentsToPoint(const Geometry::Circle& c, const Vector2<>& p) {
  /* Solves the following system, with the circle assumed centered at the origin:
   * x^2 + y^2 = r^2
   * (x - qx)^2 + (y - qy)^2 + r^2 = qx^2 + qy^2
   *
   * Solution from MATLAB:
   * syms qx qy x y real; sym r positive;
   * sol = solve([x^2 + y^2 == r^2, (x-qx)^2 + (y-qy)^2 + r^2 == qx^2 + qy^2], [x, y]);
   * [simplify(sol.x) simplify(sol.y)]
   *
   * results in:
   * [ (r*(qx*r - qy*(qx^2 + qy^2 - r^2)^(1/2)))/(qx^2 + qy^2), (r*(qx*(qx^2 + qy^2 - r^2)^(1/2) + qy*r))/(qx^2 + qy^2)]
   * [ (r*(qx*r + qy*(qx^2 + qy^2 - r^2)^(1/2)))/(qx^2 + qy^2), r*(qy*r - qx*(qx^2 + qy^2 - r^2)^(1/2))/(qx^2 + qy^2)]
   */

  // Translate p such that circle is at origin
  const Vector2<> q = p - c.center;
  const float r = c.radius;
  const float q2 = q.squareAbs();
  const float r2 = r * r;

  // Degenerate cases
  if (q2 <= r2) {
    return {};
  }

  const float tmp1 = r / q2;
  const float tmp2 = std::sqrt(q2 - r2);

  return {
    {tmp1 * (q.x * r - q.y * tmp2) + c.center.x, tmp1 * (q.y * r + q.x * tmp2) + c.center.y},
    {tmp1 * (q.x * r + q.y * tmp2) + c.center.x, tmp1 * (q.y * r - q.x * tmp2) + c.center.y},
  };
}

bool Geometry::getIntersectionOfRaysFactor(const Line& ray1, const Line& ray2, float& factor) {
  float divisor = ray2.direction.x * ray1.direction.y - ray1.direction.x * ray2.direction.y;
  if (divisor == 0) {
    return false;
  }
  float k = (ray2.direction.y * ray1.base.x - ray2.direction.y * ray2.base.x - ray2.direction.x * ray1.base.y +
             ray2.direction.x * ray2.base.y) /
            divisor;
  float l = (ray1.direction.y * ray1.base.x - ray1.direction.y * ray2.base.x - ray1.direction.x * ray1.base.y +
             ray1.direction.x * ray2.base.y) /
            divisor;
  if ((k >= 0) && (l >= 0) && (k <= 1) && (l <= 1)) {
    factor = k;
    return true;
  }
  return false;
}

float Geometry::getDistanceToLine(const Line& line, const Vector2<>& point) {
  if (line.direction.x == 0 && line.direction.y == 0) {
    return distance(point, line.base);
  }

  Vector2<> normal;
  normal.x = line.direction.y;
  normal.y = -line.direction.x;
  normal.normalize();

  float c = normal * line.base;

  return normal * point - c;
}

float Geometry::getDistanceToEdge(const Line& line, const Vector2<>& point) {
  if (line.direction.x == 0 && line.direction.y == 0) {
    return distance(point, line.base);
  }

  float c = line.direction * line.base;

  float d = (line.direction * point - c) / (line.direction * line.direction);

  if (d < 0) {
    return distance(point, line.base);
  } else if (d > 1.0f) {
    return distance(point, line.base + line.direction);
  } else {
    return std::abs(getDistanceToLine(line, point));
  }
}

float Geometry::distance(const Vector2<>& point1, const Vector2<>& point2) {
  return (point2 - point1).abs();
}

float Geometry::distance(const Vector2<int>& point1, const Vector2<int>& point2) {
  return (float)(point2 - point1).abs();
}

bool Geometry::clipLineWithQuadrangle(const Line& lineToClip,
                                      const Vector2<>& corner0,
                                      const Vector2<>& corner1,
                                      const Vector2<>& corner2,
                                      const Vector2<>& corner3,
                                      Vector2<int>& clipPoint1,
                                      Vector2<int>& clipPoint2) {
  Vector2<> point1, point2;
  bool toReturn = clipLineWithQuadrangle(lineToClip, corner0, corner1, corner2, corner3, point1, point2);
  clipPoint1.x = (int)point1.x;
  clipPoint1.y = (int)point1.y;
  clipPoint2.x = (int)point2.x;
  clipPoint2.y = (int)point2.y;
  return toReturn;
}

bool Geometry::clipLineWithQuadrangle(const Line& lineToClip,
                                      const Vector2<>& corner0,
                                      const Vector2<>& corner1,
                                      const Vector2<>& corner2,
                                      const Vector2<>& corner3,
                                      Vector2<>& clipPoint1,
                                      Vector2<>& clipPoint2) {
  Geometry::Line side[4], verticalLine;

  verticalLine.base = lineToClip.base;

  verticalLine.direction.x = -lineToClip.direction.y;
  verticalLine.direction.y = lineToClip.direction.x;

  Vector2<> corner[4];
  corner[0] = corner0;
  corner[1] = corner1;
  corner[2] = corner2;
  corner[3] = corner3;

  side[0].base = corner0;
  side[0].direction = corner1;

  side[1].base = corner1;
  side[1].direction = corner3;

  side[2].base = corner2;
  side[2].direction = corner1;

  side[3].base = corner3;
  side[3].direction = corner3;

  Vector2<> point1, point2, point;
  bool nextIsPoint1 = true;

  if (Geometry::getIntersectionOfLines(side[0], lineToClip, point)) {
    if (corner[0].x < point.x && point.x < corner[1].x) {
      if (nextIsPoint1) {
        point1 = point;
        nextIsPoint1 = false;
      }
    }
  }

  if (Geometry::getIntersectionOfLines(side[1], lineToClip, point)) {
    if (corner[1].y < point.y && point.y < corner[2].y) {
      if (nextIsPoint1) {
        point1 = point;
        nextIsPoint1 = false;
      } else {
        point2 = point;
      }
    }
  }

  if (Geometry::getIntersectionOfLines(side[2], lineToClip, point)) {
    if (corner[2].x > point.x && point.x > corner[3].x) {
      if (nextIsPoint1) {
        point1 = point;
        nextIsPoint1 = false;
      } else {
        point2 = point;
      }
    }
  }

  if (Geometry::getIntersectionOfLines(side[3], lineToClip, point)) {
    if (corner[3].y > point.y && point.y > corner[0].y) {
      if (nextIsPoint1) {
        point1 = point;
        nextIsPoint1 = false;
      } else {
        point2 = point;
      }
    }
  }

  if (nextIsPoint1) {
    return false;
  }

  if (getDistanceToLine(verticalLine, point1) < getDistanceToLine(verticalLine, point2)) {
    clipPoint1 = point1;
    clipPoint2 = point2;
  } else {
    clipPoint1 = point2;
    clipPoint2 = point1;
  }
  return true;
}

bool Geometry::isPointInsideRectangle(const Vector2<>& bottomLeftCorner,
                                      const Vector2<>& topRightCorner,
                                      const Vector2<>& point) {
  return (bottomLeftCorner.x <= point.x && point.x <= topRightCorner.x && bottomLeftCorner.y <= point.y &&
          point.y <= topRightCorner.y);
}

bool Geometry::isPointInsideRectangle2(const Vector2<>& corner1, const Vector2<>& corner2, const Vector2<>& point) {
  Vector2<> bottomLeft(std::min(corner1.x, corner2.x), std::min(corner1.y, corner2.y));
  Vector2<> topRight(std::max(corner1.x, corner2.x), std::max(corner1.y, corner2.y));
  return isPointInsideRectangle(bottomLeft, topRight, point);
}

bool Geometry::isPointInsideRectangle(const Vector2<int>& bottomLeftCorner,
                                      const Vector2<int>& topRightCorner,
                                      const Vector2<int>& point) {
  return (bottomLeftCorner.x <= point.x && point.x <= topRightCorner.x && bottomLeftCorner.y <= point.y &&
          point.y <= topRightCorner.y);
}

bool Geometry::isPointInsideConvexPolygon(const Vector2<> polygon[], const int numberOfPoints, const Vector2<>& point) {
  int orientation(ccw(polygon[0], polygon[1], point));
  if (orientation == 0) {
    return true;
  }
  for (int i = 1; i < numberOfPoints; i++) {
    int currentOrientation(ccw(polygon[i], polygon[(i + 1) % numberOfPoints], point));
    if (currentOrientation == 0) {
      return true;
    }
    if (currentOrientation != orientation) {
      return false;
    }
  }
  return true;
}

bool Geometry::checkIntersectionOfLines(const Vector2<>& l1p1,
                                        const Vector2<>& l1p2,
                                        const Vector2<>& l2p1,
                                        const Vector2<>& l2p2) {
  return (((ccw(l1p1, l1p2, l2p1) * ccw(l1p1, l1p2, l2p2)) <= 0) && ((ccw(l2p1, l2p2, l1p1) * ccw(l2p1, l2p2, l1p2)) <= 0));
}

bool Geometry::getIntersectionPointsOfLineAndRectangle(const Vector2<int>& bottomLeft,
                                                       const Vector2<int>& topRight,
                                                       const Geometry::Line line,
                                                       Vector2<int>& point1,
                                                       Vector2<int>& point2) {
  int foundPoints = 0;
  Vector2<> point[2];
  if (line.direction.x != 0) {
    float y1 = line.base.y + (bottomLeft.x - line.base.x) * line.direction.y / line.direction.x;
    if ((y1 >= bottomLeft.y) && (y1 <= topRight.y)) {
      point[foundPoints].x = (float)bottomLeft.x;
      point[foundPoints++].y = y1;
    }
    float y2 = line.base.y + (topRight.x - line.base.x) * line.direction.y / line.direction.x;
    if ((y2 >= bottomLeft.y) && (y2 <= topRight.y)) {
      point[foundPoints].x = (float)topRight.x;
      point[foundPoints++].y = y2;
    }
  }
  if (line.direction.y != 0) {
    float x1 = line.base.x + (bottomLeft.y - line.base.y) * line.direction.x / line.direction.y;
    if ((x1 >= bottomLeft.x) && (x1 <= topRight.x) && (foundPoints < 2)) {
      point[foundPoints].x = x1;
      point[foundPoints].y = (float)bottomLeft.y;
      if ((foundPoints == 0) || ((point[0] - point[1]).abs() > 0.1)) {
        foundPoints++;
      }
    }
    float x2 = line.base.x + (topRight.y - line.base.y) * line.direction.x / line.direction.y;
    if ((x2 >= bottomLeft.x) && (x2 <= topRight.x) && (foundPoints < 2)) {
      point[foundPoints].x = x2;
      point[foundPoints].y = (float)topRight.y;
      if ((foundPoints == 0) || ((point[0] - point[1]).abs() > 0.1)) {
        foundPoints++;
      }
    }
  }
  switch (foundPoints) {
  case 1:
    point1.x = (int)point[0].x;
    point2.x = point1.x;
    point1.y = (int)point[0].y;
    point2.y = point1.y;
    foundPoints++;
    return true;
  case 2:
    if ((point[1] - point[0]) * line.direction > 0) {
      point1.x = (int)point[0].x;
      point1.y = (int)point[0].y;
      point2.x = (int)point[1].x;
      point2.y = (int)point[1].y;
    } else {
      point1.x = (int)point[1].x;
      point1.y = (int)point[1].y;
      point2.x = (int)point[0].x;
      point2.y = (int)point[0].y;
    }
    return true;
  default:
    return false;
  }
}

bool Geometry::getIntersectionPointsOfLineAndRectangle(
  const Vector2<>& bottomLeft, const Vector2<>& topRight, const Geometry::Line line, Vector2<>& point1, Vector2<>& point2) {
  int foundPoints = 0;
  Vector2<> point[2];
  if (line.direction.x != 0) {
    float y1 = line.base.y + (bottomLeft.x - line.base.x) * line.direction.y / line.direction.x;
    if ((y1 >= bottomLeft.y) && (y1 <= topRight.y)) {
      point[foundPoints].x = bottomLeft.x;
      point[foundPoints++].y = y1;
    }
    float y2 = line.base.y + (topRight.x - line.base.x) * line.direction.y / line.direction.x;
    if ((y2 >= bottomLeft.y) && (y2 <= topRight.y)) {
      point[foundPoints].x = topRight.x;
      point[foundPoints++].y = y2;
    }
  }
  if (line.direction.y != 0) {
    float x1 = line.base.x + (bottomLeft.y - line.base.y) * line.direction.x / line.direction.y;
    if ((x1 >= bottomLeft.x) && (x1 <= topRight.x) && (foundPoints < 2)) {
      point[foundPoints].x = x1;
      point[foundPoints].y = bottomLeft.y;
      if ((foundPoints == 0) || ((point[0] - point[1]).abs() > 0.1)) {
        foundPoints++;
      }
    }
    float x2 = line.base.x + (topRight.y - line.base.y) * line.direction.x / line.direction.y;
    if ((x2 >= bottomLeft.x) && (x2 <= topRight.x) && (foundPoints < 2)) {
      point[foundPoints].x = x2;
      point[foundPoints].y = topRight.y;
      if ((foundPoints == 0) || ((point[0] - point[1]).abs() > 0.1)) {
        foundPoints++;
      }
    }
  }
  switch (foundPoints) {
  case 1:
    point1.x = point[0].x;
    point2.x = point1.x;
    point1.y = point[0].y;
    point2.y = point1.y;
    foundPoints++;
    return true;
  case 2:
    if ((point[1] - point[0]) * line.direction > 0) {
      point1.x = point[0].x;
      point1.y = point[0].y;
      point2.x = point[1].x;
      point2.y = point[1].y;
    } else {
      point1.x = point[1].x;
      point1.y = point[1].y;
      point2.x = point[0].x;
      point2.y = point[0].y;
    }
    return true;
  default:
    return false;
  }
}
#define CLIPLEFT 1  // 0001
#define CLIPRIGHT 2 // 0010
#define CLIPLOWER 4 // 0100
#define CLIPUPPER 8 // 1000

bool Geometry::clipLineWithRectangle(const Vector2<int>& topLeft,
                                     const Vector2<int>& bottomRight,
                                     Vector2<int>& point1,
                                     Vector2<int>& point2) {
  int K1 = 0, K2 = 0;
  int dx, dy;

  dx = point2.x - point1.x;
  dy = point2.y - point1.y;

  if (point1.y < topLeft.y) {
    K1 = CLIPLOWER;
  }
  if (point1.y > bottomRight.y) {
    K1 = CLIPUPPER;
  }
  if (point1.x < topLeft.x) {
    K1 |= CLIPLEFT;
  }
  if (point1.x > bottomRight.x) {
    K1 |= CLIPRIGHT;
  }

  if (point2.y < topLeft.y) {
    K2 = CLIPLOWER;
  }
  if (point2.y > bottomRight.y) {
    K2 = CLIPUPPER;
  }
  if (point2.x < topLeft.x) {
    K2 |= CLIPLEFT;
  }
  if (point2.x > bottomRight.x) {
    K2 |= CLIPRIGHT;
  }

  while (K1 || K2) {
    if (K1 & K2) {
      return false;
    }

    if (K1) {
      if (K1 & CLIPLEFT) {
        point1.y += (topLeft.x - point1.x) * dy / dx;
        point1.x = topLeft.x;
      } else if (K1 & CLIPRIGHT) {
        point1.y += (bottomRight.x - point1.x) * dy / dx;
        point1.x = bottomRight.x;
      }
      if (K1 & CLIPLOWER) {
        point1.x += (topLeft.y - point1.y) * dx / dy;
        point1.y = topLeft.y;
      } else if (K1 & CLIPUPPER) {
        point1.x += (bottomRight.y - point1.y) * dx / dy;
        point1.y = bottomRight.y;
      }
      K1 = 0;

      if (point1.y < topLeft.y) {
        K1 = CLIPLOWER;
      }
      if (point1.y > bottomRight.y) {
        K1 = CLIPUPPER;
      }
      if (point1.x < topLeft.x) {
        K1 |= CLIPLEFT;
      }
      if (point1.x > bottomRight.x) {
        K1 |= CLIPRIGHT;
      }
    }

    if (K1 & K2) {
      return false;
    }

    if (K2) {
      if (K2 & CLIPLEFT) {
        point2.y += (topLeft.x - point2.x) * dy / dx;
        point2.x = topLeft.x;
      } else if (K2 & CLIPRIGHT) {
        point2.y += (bottomRight.x - point2.x) * dy / dx;
        point2.x = bottomRight.x;
      }
      if (K2 & CLIPLOWER) {
        point2.x += (topLeft.y - point2.y) * dx / dy;
        point2.y = topLeft.y;
      } else if (K2 & CLIPUPPER) {
        point2.x += (bottomRight.y - point2.y) * dx / dy;
        point2.y = bottomRight.y;
      }
      K2 = 0;

      if (point2.y < topLeft.y) {
        K2 = CLIPLOWER;
      }
      if (point2.y > bottomRight.y) {
        K2 = CLIPUPPER;
      }
      if (point2.x < topLeft.x) {
        K2 |= CLIPLEFT;
      }
      if (point2.x > bottomRight.x) {
        K2 |= CLIPRIGHT;
      }
    }
  }
  return true;
}
