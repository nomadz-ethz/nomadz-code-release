#include "nomadz_core/geometry/line.hpp"

namespace nomadz_core {

  float getDistanceToLineSigned(const Line& line, const Eigen::Vector2f& point) {
    if (line.direction.x() == 0 && line.direction.y() == 0) {
      return (point - line.base).norm();
    }

    Eigen::Vector2f normal;
    normal.x() = line.direction.y();
    normal.y() = -line.direction.x();
    normal.normalize();

    const float c = normal.dot(line.base);

    return normal.dot(point) - c;
  }

  float getDistanceToLine(const Line& line, const Eigen::Vector2f& point) {
    return std::abs(getDistanceToLineSigned(line, point));
  }
} // namespace nomadz_core
