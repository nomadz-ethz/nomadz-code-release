/**
 * @file BHMath.h
 *
 * This contains some often used mathematical definitions and functions.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>,
 * <a href="mailto:alexists@tzi.de">Alexis Tsogias</a>,
 * and Max Risler
 */

#pragma once

#include "Common.h"
#include "Angle.h"
#include <type_traits>
#include <cmath>
namespace impl {
  template <typename T, bool IsSigned> struct Sgn { static constexpr int run(const T& x); };

  template <typename T> struct Sgn<T, false> {
    inline static constexpr int run(const T& x) { return T(0) < x; }
  };

  template <typename T> struct Sgn<T, true> {
    inline static constexpr int run(const T& x) { return (x > T(0)) - (x < T(0)); }
  };

  template <typename T, bool IsSigned> struct SgnPos { static constexpr int run(const T& x); };

  template <typename T> struct SgnPos<T, false> {
    inline static constexpr int run(const T& x) { return 1; }
  };

  template <typename T> struct SgnPos<T, true> {
    inline static constexpr int run(const T& x) { return (x >= T(0)) - (x < T(0)); }
  };
} // namespace impl

/**
 * Returns the sign of a value (-1, 0, or 1).
 * @param x The value.
 * @return The sign of x.
 */
template <typename T> constexpr int sgn_bh(const T& x) {
  return impl::Sgn<T, std::is_signed<T>::value>::run(x);
}

template <> constexpr int sgn_bh<Angle>(const Angle& x) {
  return sgn_bh(static_cast<float>(x));
}

/**
 * Returns the sign of a value (-1 or 1). 0 is considered to be positive.
 * @param x The value.
 * @return The sign of x.
 */
template <typename T> constexpr int sgnPos(const T& x) {
  return impl::SgnPos<T, std::is_signed<T>::value>::run(x);
}

template <> constexpr int sgnPos<Angle>(const Angle& x) {
  return sgnPos(static_cast<float>(x));
}

/**
 * Returns the sign of a value (-1 or 1). 0 is considered to be negative.
 * @param x The value.
 * @return The sign of x.
 */
template <typename T> constexpr int sgnNeg(const T& x) {
  return (x > T(0)) - (x <= T(0));
}

/**
 * Calculates the square of a value.
 * @param a The value.
 * @return The square of \c a.
 */
template <class V> constexpr V sqr_bh(const V& a) {
  return a * a;
}

inline int factorial(int number) {
  int result = 1;
  for (int i = 2; i <= number; i++) {
    result *= i;
  }
  return result;
}

template <class V> V angleAverage(const V& angle1, const V& angle2) {
  V result = (angle1 + angle2) / 2;
  if (std::abs(angle1 - angle2) > pi) {
    result = Angle::normalize(result + pi);
  }
  return result;
}
