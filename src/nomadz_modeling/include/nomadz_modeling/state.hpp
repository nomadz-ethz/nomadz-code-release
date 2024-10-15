#pragma once

#include <Eigen/Core>

namespace nomadz_modeling {
  struct State {
    enum Type { MOVING, STATIONARY };

    Type type;
    float gain;
    float weight;
    float height;
    int age;
    Eigen::Vector4f moving_x_;
    Eigen::Matrix4f moving_cov_;

    Eigen::Vector2f stationary_x_;
    Eigen::Matrix2f stationary_cov_;
  };
} // namespace nomadz_modeling
