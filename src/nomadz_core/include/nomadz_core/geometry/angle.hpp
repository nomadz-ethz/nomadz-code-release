#pragma once

#include <cmath>

namespace nomadz_core::angle {
  /**
   * @brief Normalizes an angle to the range [-pi, pi].
   *
   * This function takes an angle in radians and normalizes it to the range [-pi, pi].
   * If the input angle is outside this range, it wraps it around to the equivalent angle within the range.
   *
   * @param angle The angle to be normalized.
   * @return The normalized angle in the range [-pi, pi].
   */
  inline float normalize(float angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  inline float toDegrees(float angle) {
    return static_cast<float>(angle * 180.F / M_PI);
  }

  inline float fromDegrees(float angle) {
    return static_cast<float>(angle * M_PI / 180.F);
  }
} // namespace nomadz_core::angle
