#pragma once

#include <cmath>

namespace nomadz_core::polynom {
  int inline binomialCoefficient(int n, int k) {
    int result = 1;
    for (int i = 1; i <= k; ++i) {
      result *= (n - i + 1);
      result /= i;
    }
    return result;
  }

  template <typename T> T inline bernstein(T t, int n, int i) {
    return static_cast<T>(binomialCoefficient(n, i)) * std::pow(t, static_cast<T>(i)) *
           std::pow(1 - t, static_cast<T>(n - i));
  }
} // namespace nomadz_core::polynom
