#pragma once

#include <Eigen/Core>
#include <utility>

namespace nomadz_core {

  struct Line {
    Eigen::Vector2f base;
    Eigen::Vector2f direction;

    Line() = default;
    Line(Eigen::Vector2f base, Eigen::Vector2f direction) : base(std::move(base)), direction(std::move(direction)) {}
  };

  /**
   * Computes the signed distance of a point to a line.
   * Be careful: This means that NEGATIVE distances might occur. If you need the absolute distance, put an abs() around the
   * result.
   * @param line The line.
   * @param point The point
   * @return The signed distance between point and line. BEWARE, it might be negative, depending on which side of the line
   * the point is!
   */
  float getDistanceToLineSigned(const Line& line, const Eigen::Vector2f& point);

  /**
   * Computes the absolute distance of a point to a line.
   * @param line The line.
   * @param point The point
   * @return The absolute distance between point and line.
   */
  float getDistanceToLine(const Line& line, const Eigen::Vector2f& point);

} // namespace nomadz_core
