/**
 * @file RangeNew.h
 *
 * The file defines a template class to represent RangeNews.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"

/**
 * A template class to represent RangeNews. It also defines the 13 Allen relations
 */
template <typename T>
STREAMABLE(RangeNew, {
public:
  /**
   * Constructor.
   * Defines an empty RangeNew.
   */
  // RangeNew();

  /**
   * Constructor.
   * Defines an empty RangeNew.
   * @param minmax A conjoined starting and ending point of the empty RangeNew.
   */
  RangeNew(T minmax);

  /**
   * Constructor.
   * @param min The minimum of the RangeNew.
   * @param max The maximum of the RangeNew.
   */
  RangeNew(T min, T max);

  /**
   * The function enlarges the RangeNew so that a certain value will be part of it.
   * @param t The value that will be part of the RangeNew.
   * @return A reference to the RangeNew.
   */
  RangeNew<T>& add(T t) {
    if (min > t)
      min = t;
    if (max < t)
      max = t;
    return *this;
  }

  /**
   * The function enlarges the RangeNew so that the resulting RangeNew also contains another one.
   * @param r The RangeNew that also will be part of the RangeNew.
   * @return A reference to the RangeNew.
   */
  RangeNew<T>& add(const RangeNew<T>& r) {
    add(r.min);
    add(r.max);
    return *this;
  }

  /** A RangeNew between 0 and 1. */
  static const RangeNew<T>& ZeroOneRangeNew();

  /** A RangeNew between -1 and 1. */
  static const RangeNew<T>& OneRangeNew();

  /**
   * The function checks whether a certain value is in the RangeNew.
   * Note that the function is able to handle circular RangeNew, i.e. max < min.
   * @param t The value.
   * @return Is the value inside the RangeNew?
   */
  bool isInside(T t) const { return min <= max ? t >= min && t <= max : t >= min || t <= max; }

  /**
   * The function limits a certain value to the RangeNew.
   * Note that the function is not able to handle circular RangeNew, i.e. max < min.
   * @param t The value that will be "clipped" to the RangeNew.
   * @return The limited value.
   */
  T limit(T t) const { return t < min ? min : t > max ? max : t; } // sets a limit for a RangeNew

  /**
   * The function limits another RangeNew to this RangeNew.
   * Note that the function is able to handle circular RangeNew, i.e. max < min.
   * @param r The RangeNew that will be "clipped" to this RangeNew.
   * @return The limited value.
   */
  RangeNew<T> limit(const RangeNew<T>& r) const {
    return RangeNew<T>(limit(r.min), limit(r.max));
  } // sets the limit of a RangeNew

  /**
   * The function returns the size of the RangeNew.
   * @return The difference between the lower limit and the higher limit.
   */
  T getSize() const { return max - min; }

  /**
   * The function returns the center of the RangeNew.
   * @return The center.
   */
  T getCenter() const { return (max + min) / 2; }

  //!@name The 13 Allen relations
  //!@{
  bool operator==(const RangeNew<T>& r) const { return min == r.min && max == r.max; }
  bool operator<(const RangeNew<T>& r) const { return max < r.min; }
  bool operator>(const RangeNew<T>& r) const { return min > r.max; }
  bool meets(const RangeNew<T>& r) const { return max == r.min; }
  bool metBy(const RangeNew<T>& r) const { return min == r.max; }
  bool overlaps(const RangeNew<T>& r) const { return min < r.min && max < r.max && max > r.min; }
  bool overlappedBy(const RangeNew<T>& r) const { return min > r.min && max > r.max && min < r.max; }
  bool starts(const RangeNew<T>& r) const { return min == r.min && max < r.max; }
  bool startedBy(const RangeNew<T>& r) const { return min == r.min && max > r.max; }
  bool finishes(const RangeNew<T>& r) const { return max == r.max && min > r.min; }
  bool finishedBy(const RangeNew<T>& r) const { return max == r.max && min < r.min; }
  bool during(const RangeNew<T>& r) const { return min > r.min && max < r.max; }
  bool contains(const RangeNew<T>& r) const { return min < r.min && max > r.max; }
  //!@}

  // The size of the intersection of to RangeNews or 0 if there is no intersection
  T intersectionSizeWith(const RangeNew<T>& r) const { return std::max(0.f, std::min(max, r.max) - std::max(min, r.min)); }
  ,

    (T)min, (T)max, /**< The limits of the RangeNew. */
});

// template<typename T> RangeNew<T>::RangeNew() : min(T()), max(T()) {}
template <typename T> RangeNew<T>::RangeNew(T minmax) : min(minmax), max(minmax) {}
template <typename T> RangeNew<T>::RangeNew(T min, T max) : min(min), max(max) {}

template <typename T> const RangeNew<T>& RangeNew<T>::ZeroOneRangeNew() {
  static RangeNew<T> RangeNew(0, 1);
  return RangeNew;
}

template <typename T> const RangeNew<T>& RangeNew<T>::OneRangeNew() {
  static RangeNew<T> RangeNew(-1, 1);
  return RangeNew;
}

using RangeNewi = RangeNew<int>;
using RangeNewf = RangeNew<float>;
