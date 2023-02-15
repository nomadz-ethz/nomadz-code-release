/**
 * @file RingBufferWithSumNew.h
 *
 * The file declares a ring buffer that can determine the sum and average of its
 * elements in constant time. The type of the elements must be assignable, copyable,
 * and must support addition with itself and division by an integral (average()) or
 * floating point (averagef()) number. The interface of the class is similar to
 * types of the standard template library and it also supports for-each loops.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Thomas Röfer
 */

#pragma once

#include "RingBufferNew.h"
#include <type_traits>

template <typename T, std::size_t n = 0> class RingBufferWithSumNew : public RingBufferNew<T, n> {
private:
  T zero;       /**< Sum when the buffer is empty. */
  T currentSum; /**< Sum of current round since index 0. */
  T prevSum;    /** Sum of previous round. */

public:
  /**
   * Constructor for a buffer of entries with a default constructor that creates a
   * zero element.
   * @param capacity The maximum number of entries the buffer can store. If not specified,
   *                 the second template parameter is used as default capacity.
   */
  RingBufferWithSumNew(std::size_t capacity = n) : RingBufferNew<T, n>(capacity), zero() {
    static_assert(std::is_same<Angle, T>::value || std::is_floating_point<T>::value || std::is_integral<T>::value,
                  "The standard constructor is not applicable for non-standard types in the template argument");
    clear();
  }

  /**
   * Constructor for a buffer of entries without a default constructor that creates a
   * zero element, e.g. Eigen vectors.
   * @param zero A value that is used as zero element.
   * @param capacity The maximum number of entries the buffer can store. If not specified,
   *                 the second template parameter is used as default capacity.
   */
  RingBufferWithSumNew(const T& zero, std::size_t capacity = n) : RingBufferNew<T, n>(capacity), zero(zero) {
    static_assert(!std::is_same<Angle, T>::value && !std::is_floating_point<T>::value && !std::is_integral<T>::value,
                  "Use the standard constructor instead!");
    clear();
  }

  /**
   * Copy constructor.
   * If the buffer is full, the previous sum contains all data, otherwise the current
   * sum contains all data.
   * @param other The buffer that is copied.
   */
  RingBufferWithSumNew(const RingBufferWithSumNew& other)
      : zero(other.zero), currentSum(RingBufferNew<T, n>::full() ? other.zero : other.prevSum + other.currentSum),
        prevSum(RingBufferNew<T, n>::full() ? other.prevSum + other.currentSum : other.zero) {}

  /**
   * Assignment operator.
   * If the buffer is full, the previous sum contains all data, otherwise the current
   * sum contains all data.
   * @param other The buffer that is copied.
   * @return This buffer.
   */
  RingBufferWithSumNew& operator=(const RingBufferWithSumNew& other) {
    RingBufferNew<T, n>::operator=(other);
    zero = other.zero;
    currentSum = RingBufferNew<T, n>::full() ? other.zero : other.prevSum + other.currentSum;
    prevSum = RingBufferNew<T, n>::full() ? other.prevSum + other.currentSum : other.zero;
    return *this;
  }

  /** Empties the buffer. */
  void clear() {
    RingBufferNew<T, n>::clear();
    currentSum = prevSum = zero;
  }

  /**
   * Changes the capacity of the buffer. If it actually changes, the complexity is O(size()).
   * @param capacity The maximum number of entries the buffer can store.
   */
  void reserve(std::size_t capacity) {
    while (RingBufferNew<T, n>::size() > capacity)
      pop_back();
    RingBufferNew<T, n>::reserve(capacity);
  }

  /**
   * Adds a new entry to the front of the buffer. The new entry is accessible under
   * index 0, front(), and *begin(). If the buffer was already full, the entry at
   * back() is lost.
   * @param value The value that is added to the buffer.
   */
  void push_front(const T& value) {
    if (RingBufferNew<T, n>::full())
      prevSum -= RingBufferNew<T, n>::back();

    currentSum += value;
    RingBufferNew<T, n>::push_front(value);

    // Prevent propagating errors from one round to another
    if (RingBufferNew<T, n>::cycled()) {
      prevSum = currentSum;
      currentSum = zero;
    }
  }

  /**
   * Replaces the entry at the front of the buffer. The new entry is still accessible under
   * index 0, front(), and *begin(). All other entries are unchanged.
   * @param newValue The value that is set to the front of the buffer.
   */
  void set_front(const T& newValue) {
    if (RingBufferNew<T, n>::cycled())
      prevSum += -RingBufferNew<T, n>::front() + newValue;
    else
      currentSum += -RingBufferNew<T, n>::front() + newValue;

    RingBufferNew<T, n>::front() = newValue;
  }

  /** Removes the entry back() from the buffer. */
  void pop_back() {
    prevSum -= RingBufferNew<T, n>::back();
    RingBufferNew<T, n>::pop_back();
  }

  /** Returns the sum of all entries in O(1). */
  T sum() const { return prevSum + currentSum; }

  /**
   * Returns the minimum of all entries in O(size()).
   * If the buffer is empty, the zero element is returned.
   */
  T minimum() const {
    if (RingBufferNew<T, n>::empty())
      return zero;
    else {
      T min = RingBufferNew<T, n>::front();
      for (const T& t : *this)
        if (t < min)
          min = t;
      return min;
    }
  }

  /**
   * Returns the maximum of all entries in O(size()).
   * If the buffer is empty, the zero element is returned.
   */
  T maximum() const {
    if (RingBufferNew<T, n>::empty())
      return zero;
    else {
      T max = RingBufferNew<T, n>::front();
      for (const T& t : *this)
        if (t > max)
          max = t;
      return max;
    }
  }

  /**
   * Returns the average of all entries in O(1). If the buffer is empty, the
   * zero element is returned.
   */
  T average() const;

  /**
   * Returns the average of all entries in O(1) as a float. If the buffer is empty, the
   * zero element is returned. This method is only useful if T is an integral type.
   */
  float averagef() const;

  /**
   * fills/overwrites the buffer with given value
   * @param value value to fill the buffer with
   */
  void fill(T value) {
    for (std::size_t i = 0; i < n; i++)
      push_front(value);
  }

  /**
   * returns the variance of all entries
   * \return the variance
   */
  inline T getVariance() {
    // Return 0 if buffer is empty or has just one element
    if (RingBufferNew<T, n>::size() < 2)
      return T();
    T avg = average();
    T var = 0;

    for (const T& t : *this) {
      var += (t - avg) * (t - avg);
    }

    return (1.f / (RingBufferNew<T, n>::size() - 1)) * var;
  }
};

#ifdef WINDOWS // The division might force a type conversion that looses precision
#pragma warning(push)
#pragma warning(disable : 4244 4267)
#elif defined __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif

template <typename T, std::size_t n> T RingBufferWithSumNew<T, n>::average() const {
  if (RingBufferNew<T, n>::empty())
    return zero;
  else
    return static_cast<T>(sum() / RingBufferNew<T, n>::size());
}

template <typename T, std::size_t n> float RingBufferWithSumNew<T, n>::averagef() const {
  static_assert(std::is_integral<T>::value, "Use average() instead");
  if (RingBufferNew<T, n>::empty())
    return zero;
  else
    return sum() / static_cast<float>(RingBufferNew<T, n>::size());
}

#ifdef WINDOWS
#pragma warning(pop)
#elif defined __clang__
#pragma clang diagnostic pop
#endif
