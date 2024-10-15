#pragma once

#include <Eigen/Core>

namespace nomadz_core {
  struct Circle {
    Circle() = default;
    Circle(Eigen::Vector2f c, float r) : center(std::move(c)), radius(r){};

    Eigen::Vector2f center = Eigen::Vector2f::Zero();
    float radius = 0.0;
  };
} // namespace nomadz_core
