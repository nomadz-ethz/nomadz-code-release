/**
 * @file RingBuffer.h
 *
 * Declaration of class RingBuffer
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Max Risler
 */

#include <cstddef>
#include <cstring>
#include "Core/System/BHAssert.h"
#include "Core/System/SystemCall.h"
#pragma once

/**
 * @class RingBuffer
 *
 * template class for cyclic buffering of the last n values of Type V
 */
template <class V, int n> class RingBuffer {
public:
  /** Constructor */
  RingBuffer() { init(); }

  /**
   * initializes the Ringbuffer
   */
  class iterator {
  private:
    RingBuffer<V, n>& buffer; /**< The buffer. */
    std::size_t index;        /**< Index of the current entry. */

  public:
    iterator(RingBuffer<V, n>& buffer, std::size_t index) : buffer(buffer), index(index) {}
    V* operator->() const { return &buffer[index]; }
    V& operator*() const { return buffer[index]; }
    bool operator==(const iterator& other) const { return index == other.index; }
    bool operator!=(const iterator& other) const { return index != other.index; }
    iterator operator++() {
      ++index;
      return *this;
    }
    iterator operator++(int) {
      iterator result(*this);
      ++index;
      return result;
    }
    iterator operator+=(std::ptrdiff_t offset) {
      index += offset;
      return *this;
    }
    iterator operator-=(std::ptrdiff_t offset) {
      index -= offset;
      return *this;
    }
    iterator operator+(std::ptrdiff_t offset) {
      iterator result(*this);
      return result += offset;
    }
    iterator operator-(std::ptrdiff_t offset) {
      iterator result(*this);
      return result -= offset;
    }
  };

  /** A class for constant iterators with its typical interface. */
  class const_iterator {
  private:
    const RingBuffer<V, n>& buffer; /**< The buffer. */
    std::size_t index;              /**< Index of the current entry. */

  public:
    const_iterator(const RingBuffer<V, n>& buffer, std::size_t index) : buffer(buffer), index(index) {}
    const V* operator->() const { return &buffer[index]; }
    const V& operator*() const { return buffer[index]; }
    bool operator==(const const_iterator& other) const { return index == other.index; }
    bool operator!=(const const_iterator& other) const { return !(*this == other); }
    const_iterator operator++() {
      ++index;
      return *this;
    }
    const_iterator operator++(int) {
      iterator result(*this);
      ++index;
      return result;
    }
    const_iterator operator+=(std::ptrdiff_t offset) {
      index += offset;
      return *this;
    }
    const_iterator operator-=(std::ptrdiff_t offset) {
      index -= offset;
      return *this;
    }
    const_iterator operator+(std::ptrdiff_t offset) {
      iterator result(*this);
      return result += offset;
    }
    const_iterator operator-(std::ptrdiff_t offset) {
      iterator result(*this);
      return result -= offset;
    }
  };

  inline void init() {
    current = n - 1;
    numberOfEntries = 0;
  }

  /**
   * adds an entry to the buffer
   * \param v value to be added
   */
  inline void add(const V& v) {
    add();
    buffer[current] = v;
  }

  /**
   * adds an entry to the buffer.
   * The new head is not initialized, but can be changed afterwards.
   */
  inline void add() {
    current++;
    current %= n;
    if (++numberOfEntries >= n)
      numberOfEntries = n;
  }

  /**
   * removes the first added entry to the buffer
   */
  inline void removeFirst() { --numberOfEntries; }

  /**
   * returns an entry
   * \param i index of entry counting from last added (last=0,...)
   * \return a reference to the buffer entry
   */
  inline V& getEntry(int i) { return (*this)[i]; }

  /**
   * returns an const entry
   * \param i index of entry counting from last added (last=0,...)
   * \return a reference to the buffer entry
   */
  inline const V& getEntry(int i) const { return (*this)[i]; }

  /**
   * returns an entry
   * \param i index of entry counting from last added (last=0,...)
   * \return a reference to the buffer entry
   */
  inline V& operator[](int i) {
    ASSERT(!empty());
    auto idx = (n + current - i) % n;
    if (idx >= numberOfEntries) {
      return buffer[0];
    }
    return buffer[idx];
  }

  /**
   * returns a constant entry.
   * \param i index of entry counting from last added (last=0,...)
   * \return a reference to the buffer entry
   */
  inline const V& operator[](int i) const {
    ASSERT(!empty());
    auto idx = (n + current - i) % n;
    if (idx >= numberOfEntries) {
      return buffer[0];
    }
    return buffer[idx];
  }

  /** Returns the number of elements that are currently in the ring buffer
   * \return The number
   */
  inline int getNumberOfEntries() const { return numberOfEntries; }

  /**
   * Returns the maximum entry count.
   * \return The maximum entry count.
   */
  inline int getMaxEntries() const { return n; }

  /**
   * Determines whether maximum entry count equals actual number of entries.
   * @return true iff getMaxEntries == getNumberOfEntries.
   */
  inline bool isFilled() const { return getMaxEntries() == getNumberOfEntries(); }

  /**
   * Determines whether the buffer is empty.
   * \return True, if the number of entries is 0.
   */
  inline bool isEmpty() const { return !numberOfEntries; }

  bool full() const { return numberOfEntries == n; }

  /** The number of elements currently stored in the buffer. */
  std::size_t size() const { return numberOfEntries; }
  inline bool empty() const { return isEmpty(); }

  /** Access the the last element of the buffer. */
  V& back() {
    ASSERT(!empty());
    return (*this)[0];
  }
  const V& back() const {
    ASSERT(!empty());
    return (*this)[0];
  }

private:
  int current;
  int numberOfEntries;
  V buffer[n];
};
