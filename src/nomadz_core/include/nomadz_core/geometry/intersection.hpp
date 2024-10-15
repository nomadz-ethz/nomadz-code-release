#pragma once

#include <cmath>

#include "nomadz_core/geometry/line.hpp"
#include "nomadz_core/geometry/circle.hpp"

#include "nomadz_core/math/bh_math.hpp"

namespace nomadz_core {

  inline int getIntersectionOfLineAndCircle(const Line& line,
                                            const Circle& circle,
                                            Eigen::Vector2f& firstIntersection,
                                            Eigen::Vector2f& secondIntersection) {
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
    const float divisor = line.direction.squaredNorm();
    const float p = 2 * (line.base.dot(line.direction) - circle.center.dot(line.direction)) / divisor;
    const float q = ((line.base - circle.center).squaredNorm() - sqr(circle.radius)) / divisor;
    const float p_2 = p / 2.0F;
    const float radicand = sqr(p_2) - q;
    if (radicand < 0) {
      return 0;
    }
    const float radix = std::sqrt(radicand);
    firstIntersection = line.base + line.direction * (-p_2 + radix);
    secondIntersection = line.base + line.direction * (-p_2 - radix);
    return radicand == 0 ? 1 : 2;
  }

} // namespace nomadz_core
