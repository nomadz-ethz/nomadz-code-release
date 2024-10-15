#pragma once

#include <cmath>
#include <limits>
#include <algorithm>

#include "nomadz_core/math/bh_math.hpp"
#include "nomadz_core/math/constants.hpp"

namespace nomadz_core::approx {

  template <typename T> bool isZero(T a, T prec = std::numeric_limits<T>::epsilon()) {
    using std::abs;
    return abs(a) < prec;
  }

  template <typename T> bool isEqual(T a, T b, T prec = std::numeric_limits<T>::epsilon()) {
    using std::abs;
    using std::max;
    const T diff = abs(a - b);
    return diff < prec || diff < prec * max(abs(a), abs(b));
  }

  /**
   * An approximation of atan2 with an error < 0.005f.
   * 3-5x times faster than atan2 from cmath
   */
  inline float atan2(float y, float x) {
    if (x == 0.F) {
      return static_cast<float>(sgn(y)) * constants::PI / 2.F;
    }
    const float z = y / x;
    if (std::abs(z) < 1.F) {
      const float atan = z / (1.F + 0.28F * z * z);
      if (x < 0.F) {
        if (y < 0.F) {
          return atan - constants::PI;
        }
        return atan + constants::PI;
      }
      return atan;
    }
    const float atan = constants::PI / 2.F - z / (z * z + 0.28F);
    if (y < 0.F) {
      return atan - constants::PI;
    }
    return atan;
  }
} // namespace nomadz_core::approx
