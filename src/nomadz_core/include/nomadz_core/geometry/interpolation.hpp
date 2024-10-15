#pragma once

#include <cmath>
#include <array>
#include <Eigen/Core>

#include "nomadz_core/math/polynom.hpp"

namespace nomadz_core::interpolation {
  template <typename VectorType, int Degree = 3>
  VectorType inline bezierInterpolation(const std::array<VectorType, Degree + 1>& control_points, float ratio) {
    static_assert(std::is_base_of<Eigen::MatrixBase<VectorType>, VectorType>::value,
                  "Template type must be an Eigen vector type");
    VectorType result = VectorType::Zero();
    for (int i = 0; i <= Degree; ++i) {
      result += control_points[i] * nomadz_core::polynom::bernstein<float>(ratio, Degree, i);
    }
    return result;
  };

  template <typename VectorType, int Degree = 3>
  VectorType inline rationalBezierInterpolation(const std::array<VectorType, Degree + 1>& control_points,
                                                const std::array<float, Degree + 1>& control_point_weights,
                                                float ratio) {
    static_assert(std::is_base_of<Eigen::MatrixBase<VectorType>, VectorType>::value,
                  "Template type must be an Eigen vector type");
    for (size_t i = 0; i < control_point_weights.size(); ++i) {
      assert(control_point_weights[i] != 0.0F && "Control point weight should not be zero!");
    }
    VectorType result = VectorType::Zero();
    float denominator = 0;
    for (int i = 0; i <= Degree; ++i) {
      result += control_points[i] * control_point_weights[i] * nomadz_core::polynom::bernstein<float>(ratio, Degree, i);
      denominator += control_point_weights[i] * nomadz_core::polynom::bernstein<float>(ratio, Degree, i);
    }
    return result / denominator;
  };
} // namespace nomadz_core::interpolation
