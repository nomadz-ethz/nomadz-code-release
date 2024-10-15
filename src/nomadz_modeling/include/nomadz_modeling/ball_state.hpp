#pragma once

#include <Eigen/Core>

namespace nomadz_modeling {
  struct BallState {
    Eigen::Vector2f position;
    Eigen::Vector2f velocity;
    Eigen::Matrix<float, 2, 2> position_covariance;
  };
} // namespace nomadz_modeling
