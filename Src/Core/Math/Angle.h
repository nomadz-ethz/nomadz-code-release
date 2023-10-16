/**
 * @file Angle.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in License.Nao_Devils.txt.
 * (c) 2023 Nao Devils and NomadZ team
 */

#pragma once

#include "Core/Enum.h"
#include "Common.h"

/** constant for a full circle*/
constexpr float pi_2f = 2.f * 3.1415926535897932384626433832795f;

/**
 * Converts angle from rad to degrees.
 * @param angle code in rad
 * @return angle coded in degrees
 */
// template <class V> constexpr V toDegrees(V angle) { return angle * V(180.f / 3.1415926535897932384626433832795f); }

/**
 * The Angle class stores the represented angle in radiant.
 */
class Angle {
private:
  float value = 0.f;

public:
  constexpr Angle() = default;
  constexpr Angle(float angle) : value(angle) {}

  operator float&() { return value; }
  constexpr operator const float&() const { return value; }

  constexpr Angle operator-() const { return -value; }
  Angle& operator+=(float angle) {
    value += angle;
    return *this;
  }
  Angle& operator-=(float angle) {
    value -= angle;
    return *this;
  }
  Angle& operator*=(float angle) {
    value *= angle;
    return *this;
  }
  Angle& operator/=(float angle) {
    value /= angle;
    return *this;
  }

  Angle& normalize() {
    value = normalize(value);
    return *this;
  }

  /**
   * reduce angle to [-pi..+pi[
   * @param data angle coded in rad
   * @return normalized angle coded in rad
   */
  template <class V> static V normalize(V data);

  static constexpr Angle fromDegrees(float degrees) { return Angle((degrees / 180.f) * 3.1415926535897932384626433832795f); }
  static constexpr Angle fromDegrees(int degrees) { return fromDegrees(static_cast<float>(degrees)); }

  float constexpr toDegrees() const { return (value / 3.1415926535897932384626433832795f) * 180.f; }
};

inline constexpr Angle operator"" _deg(unsigned long long int angle) {
  return Angle::fromDegrees(static_cast<float>(angle));
}

inline constexpr Angle operator"" _deg(long double angle) {
  return Angle::fromDegrees(static_cast<float>(angle));
}

inline constexpr Angle operator"" _rad(unsigned long long int angle) {
  return Angle(static_cast<float>(angle));
}

inline constexpr Angle operator"" _rad(long double angle) {
  return Angle(static_cast<float>(angle));
}

template <class V> inline V Angle::normalize(V data) {
  if (data >= -V(pi_2f) && data < V(3.1415926535897932384626433832795f))
    return data;
  else {
    data = data - static_cast<int>(data / V(pi_2f)) * V(pi_2f);
    return data >= V(3.1415926535897932384626433832795f)
             ? V(data - V(pi_2f))
             : data < -V(3.1415926535897932384626433832795f) ? V(data + V(pi_2f)) : data;
  }
}
