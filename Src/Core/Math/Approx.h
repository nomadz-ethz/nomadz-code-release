/**
 * @file Approx.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in License.Nao_Devils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#pragma once

#include "BHMath.h"
#include "Constants.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Approx {
  template <typename T> bool isZero(T a, T prec = std::numeric_limits<T>::epsilon()) { return std::abs(a) < prec; }

  template <typename T> bool isEqual(T a, T b, T prec = std::numeric_limits<T>::epsilon()) {
    const T diff = std::abs(a - b);
    return diff < prec || diff < prec * std::max(std::abs(a), std::abs(b));
  }
} // namespace Approx

/**
 * An approximation of atan2 with an error < 0.005f.
 * 3-5x times faster than atan2 from cmath
 */
inline float approxAtan2(float y, float x) {
  if (x == 0.f) {
    if (y > 0.0f)
      return 1.5707963f;
    if (y == 0.0f)
      return 0.0f;
    return -1.5707963f;
  }
  float atan;
  float z = y / x;
  if (fabs(z) < 1.f) {
    atan = z / (1.f + 0.28f * z * z);
    if (x < 0.0f) {
      if (y < 0.0f)
        return atan - 3.14159265f;
      return atan + 3.14159265f;
    }
  } else {
    atan = 1.5707963f - z / (z * z + 0.28f);
    if (y < 0.f)
      return atan - 3.14159265f;
  }
  return atan;
}
