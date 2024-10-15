
#pragma once

#include <limits>
#include <algorithm>
#include <Eigen/Core>

namespace nomadz_core {
  /**
   * The template class represents a range with a minimum and maximum value.
   */
  template <typename T = float> class Range {
  public:
    T min; /**< The minimum of the range. */
    T max; /**< The maximum of the range. */

    /**
     * Constructor.
     * Defines an empty range.
     */
    constexpr Range() : min(std::numeric_limits<T>::max()), max(std::numeric_limits<T>::lowest()) {}

    /**
     * Constructor.
     * Defines an empty range around a value.
     * @param minmax A conjoined starting and ending point of the empty range.
     */
    explicit constexpr Range(T minmax) : min(minmax), max(minmax) {}

    /**
     * Constructor.
     * @param min The minimum of the range.
     * @param max The maximum of the range.
     */
    constexpr Range(T min, T max) : min(min), max(max) {}

    /** A range between 0 and 1. */
    static constexpr Range<T> zeroOneRange() { return Range<T>(T(0), T(1)); }

    /** A range between -1 and 1. */
    static constexpr Range<T> oneRange() { return Range<T>(T(-1), T(1)); }

    /**
     * The function enlarges the range so that a certain value will be part of it.
     * @param t The value that will be part of the range.
     * @return A reference to the range.
     */
    Range<T>& add(T t) {
      if (min > t) {
        min = t;
      }
      if (max < t) {
        max = t;
      }
      return *this;
    }

    /**
     * The function enlarges the range so that the resulting range also contains another one.
     * @param r The range that also will be part of the range.
     * @return A reference to the range.
     */
    Range<T>& add(const Range<T>& r) {
      add(r.min);
      add(r.max);
      return *this;
    }

    /**
     * The function mirrors the range.
     */
    Range<T>& mirror() {
      const T min_val = min;
      min = -max;
      max = -min_val;
      return *this;
    }

    /**
     * The function checks whether a certain value is in the range.
     * Note that the function is able to handle circular range, i.e. max < min.
     * @param t The value.
     * @return Is the value inside the range?
     */
    constexpr bool isInside(T t) const { return min <= max ? t >= min && t <= max : t >= min || t <= max; }

    /**
     * The function limits a certain value to the range.
     * Note that the function is not able to handle circular range, i.e. max < min.
     * @param t The value that will be "clipped" to the range.
     * @return The limited value.
     */
    constexpr T limit(T t) const { return t < min ? min : t > max ? max : t; }

    constexpr T clamped(T t) const { return limit(t); }
    T& clamp(T& t) const {
      t = clamped(t);
      return t;
    }

    template <typename Derived> Derived& clamp(Eigen::DenseBase<Derived>& mat) const {
      static_assert(std::is_same<typename Eigen::MatrixBase<Derived>::Scalar, T>::value,
                    "Matrix must have the same scalar type as the Range.");
      return mat = mat.derived().unaryExpr([this](T val) { return clamped(val); });
    }

    template <typename Derived> Derived clamped(const Eigen::DenseBase<Derived>& mat) const {
      static_assert(std::is_same<typename Eigen::MatrixBase<Derived>::Scalar, T>::value,
                    "Matrix must have the same scalar type as the Range.");
      return mat.derived().unaryExpr([this](T val) { return clamped(val); });
    }

    /**
     * The function limits another range to this range.
     * Note that the function is able to handle circular range, i.e. max < min.
     * @param r The range that will be "clipped" to this range.
     * @return The limited value.
     */
    constexpr Range<T> limit(const Range<T>& r) const { return Range<T>(limit(r.min), limit(r.max)); }

    /**
     * Scales a value t with a range of tRange to this range.
     */
    T scale(T t, const Range<T>& tRange) const {
      return limit(((t - tRange.min) / (tRange.max - tRange.min)) * (max - min) + min);
    }

    /**
     * The function returns the size of the range.
     * Note that the function is not able to handle circular angle range, i.e. max < min (except for Rangea).
     * @return The difference between the lower limit and the higher limit.
     */
    constexpr T getSize() const { return max - min; }

    /**
     * The function returns the center of the range.
     * Note that the function is not able to handle circular angle range, i.e. max < min.
     * @return The center.
     */
    constexpr T getCenter() const { return (max + min) / 2; }

    //!@name The 13 Allen relations
    //!@{
    constexpr bool operator==(const Range<T>& r) const { return min == r.min && max == r.max; }
    constexpr bool operator<(const Range<T>& r) const { return max < r.min; }
    constexpr bool operator>(const Range<T>& r) const { return min > r.max; }
    constexpr bool meets(const Range<T>& r) const { return max == r.min; }
    constexpr bool metBy(const Range<T>& r) const { return min == r.max; }
    constexpr bool overlaps(const Range<T>& r) const { return min < r.min && max < r.max && max > r.min; }
    constexpr bool overlappedBy(const Range<T>& r) const { return min > r.min && max > r.max && min < r.max; }
    constexpr bool starts(const Range<T>& r) const { return min == r.min && max < r.max; }
    constexpr bool startedBy(const Range<T>& r) const { return min == r.min && max > r.max; }
    constexpr bool finishes(const Range<T>& r) const { return max == r.max && min > r.min; }
    constexpr bool finishedBy(const Range<T>& r) const { return max == r.max && min < r.min; }
    constexpr bool during(const Range<T>& r) const { return min > r.min && max < r.max; }
    constexpr bool contains(const Range<T>& r) const { return min < r.min && max > r.max; }
    //!@}

    constexpr bool operator!=(const Range<T>& r) const { return min != r.min || max != r.max; }

    // The size of the intersection of two ranges or 0 if there is no intersection
    constexpr T intersectionSizeWith(const Range<T>& r) const {
      return std::max(0.F, std::min(max, r.max) - std::max(min, r.min));
    }
  };

  using Rangef = Range<float>;
  using Rangei = Range<int>;
} // namespace nomadz_core
