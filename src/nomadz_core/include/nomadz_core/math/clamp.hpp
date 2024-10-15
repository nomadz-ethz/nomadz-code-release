#pragma once

#include <Eigen/Core>

namespace nomadz_core {
  template <typename VectorType> inline VectorType ellipsoidClamp(const VectorType& current_vec, const VectorType& ref_vec) {
    static_assert(std::is_base_of<Eigen::MatrixBase<VectorType>, VectorType>::value,
                  "Template type must be an Eigen vector type");
    assert(ref_vec.all() && "Reference vector must not contain zero values");
    VectorType scaled = current_vec.cwiseQuotient(ref_vec);
    if (scaled.norm() <= 1) {
      return current_vec;
    }
    return scaled.normalized().cwiseProduct(ref_vec);
  }

  template <typename VectorType> inline VectorType rectClamp(const VectorType& current_vec, const VectorType& ref_vec) {
    static_assert(std::is_base_of<Eigen::MatrixBase<VectorType>, VectorType>::value,
                  "Template type must be an Eigen vector type");
    return current_vec.cwiseMax(-ref_vec).cwiseMin(ref_vec);
  }

  template <typename VectorType>
  inline VectorType rectClamp(const VectorType& current_vec, const VectorType& min_vec, const VectorType& max_vec) {
    static_assert(std::is_base_of<Eigen::MatrixBase<VectorType>, VectorType>::value,
                  "Template type must be an Eigen vector type");
    return current_vec.cwiseMax(min_vec).cwiseMin(max_vec);
  }
} // namespace nomadz_core
