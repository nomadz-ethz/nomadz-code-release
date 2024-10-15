#pragma once

#include <Eigen/Core>

#include "nomadz_core/math/bh_math.hpp"
#include "nomadz_core/math/approx.hpp"

namespace nomadz_core {

  template <typename T> T getRotationRelativeXAngle(const Eigen::Matrix<T, 3, 3>& mat) {
    const float h = std::sqrt(mat(1, 1) * mat(1, 1) + mat(2, 1) * mat(2, 1));
    if (approx::isZero(h)) {
      return 0.F;
    }

    return std::acos(mat(1, 1) / h) * sgnNeg(mat(2, 1));
  }

  template <typename T> T getRotationRelativeYAngle(const Eigen::Matrix<T, 3, 3>& mat) {
    const float h = std::sqrt(mat(0, 0) * mat(0, 0) + mat(2, 0) * mat(2, 0));
    if (approx::isZero(h)) {
      return 0.F;
    }

    return std::acos(mat(0, 0) / h) * -sgnNeg(mat(2, 0));
  }

  template <typename T> T getRotationRelativeZAngle(const Eigen::Matrix<T, 3, 3>& mat) {
    const float h = std::sqrt(mat(0, 0) * mat(0, 0) + mat(1, 0) * mat(1, 0));
    if (approx::isZero(h)) {
      return 0.F;
    }

    return std::acos(mat(0, 0) / h) * sgnNeg(mat(1, 0));
  }

} // namespace nomadz_core
