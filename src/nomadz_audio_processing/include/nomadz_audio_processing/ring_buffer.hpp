#pragma once

#include <cstddef>
#include <cstring>
#include <cassert>

/**
 * @class RingBuffer
 *
 * template class for cyclic buffer_ing of the last N values of Type V
 */
template <class V, int N> class RingBuffer {
public:
  /** Constructor */
  RingBuffer() { init(); }

  /**
   * initializes the Ringbuffer_
   */
  class Iterator {
  private:
    RingBuffer<V, N>& buffer_; /**< The buffer_. */
    std::size_t index_;        /**< Index of the current entry. */

  public:
    Iterator(RingBuffer<V, N>& buffer_, std::size_t index_) : buffer_(buffer_), index_(index_) {}
    V* operator->() const { return &buffer_[index_]; }
    V& operator*() const { return buffer_[index_]; }
    bool operator==(const Iterator& other) const { return index_ == other.index_; }
    bool operator!=(const Iterator& other) const { return index_ != other.index_; }
    Iterator operator++() {
      ++index_;
      return *this;
    }
    Iterator operator++(int) {
      Iterator result(*this);
      ++index_;
      return result;
    }
    Iterator operator+=(std::ptrdiff_t offset) {
      index_ += offset;
      return *this;
    }
    Iterator operator-=(std::ptrdiff_t offset) {
      index_ -= offset;
      return *this;
    }
    Iterator operator+(std::ptrdiff_t offset) {
      Iterator result(*this);
      return result += offset;
    }
    Iterator operator-(std::ptrdiff_t offset) {
      Iterator result(*this);
      return result -= offset;
    }
  };

  /** A class for constant Iterators with its typical interface. */
  class ConstIterator {
  private:
    const RingBuffer<V, N>& buffer_; /**< The buffer_. */
    std::size_t index_;              /**< Index of the current entry. */

  public:
    ConstIterator(const RingBuffer<V, N>& buffer_, std::size_t index_) : buffer_(buffer_), index_(index_) {}
    const V* operator->() const { return &buffer_[index_]; }
    const V& operator*() const { return buffer_[index_]; }
    bool operator==(const ConstIterator& other) const { return index_ == other.index_; }
    bool operator!=(const ConstIterator& other) const { return !(*this == other); }
    ConstIterator operator++() {
      ++index_;
      return *this;
    }
    ConstIterator operator++(int) {
      Iterator result(*this);
      ++index_;
      return result;
    }
    ConstIterator operator+=(std::ptrdiff_t offset) {
      index_ += offset;
      return *this;
    }
    ConstIterator operator-=(std::ptrdiff_t offset) {
      index_ -= offset;
      return *this;
    }
    ConstIterator operator+(std::ptrdiff_t offset) {
      Iterator result(*this);
      return result += offset;
    }
    ConstIterator operator-(std::ptrdiff_t offset) {
      Iterator result(*this);
      return result -= offset;
    }
  };

  inline void init() {
    current_ = N - 1;
    number_of_entries_ = 0;
  }

  /**
   * adds an entry to the buffer_
   * \param v value to be added
   */
  inline void add(const V& v) {
    add();
    buffer_[current_] = v;
  }

  /**
   * adds an entry to the buffer_.
   * The new head is not initialized, but can be changed afterwards.
   */
  inline void add() {
    current_++;
    current_ %= N;
    if (++number_of_entries_ >= N) {
      number_of_entries_ = N;
    }
  }

  /**
   * removes the first added entry to the buffer_
   */
  inline void removeFirst() { --number_of_entries_; }

  /**
   * returns an entry
   * \param i index_ of entry counting from last added (last=0,...)
   * \return a reference to the buffer_ entry
   */
  inline V& getEntry(int i) { return (*this)[i]; }

  /**
   * returns an const entry
   * \param i index_ of entry counting from last added (last=0,...)
   * \return a reference to the buffer_ entry
   */
  inline const V& getEntry(int i) const { return (*this)[i]; }

  /**
   * returns an entry
   * \param i index_ of entry counting from last added (last=0,...)
   * \return a reference to the buffer_ entry
   */
  inline V& operator[](int i) {
    assert(!empty());
    auto idx = (N + current_ - i) % N;
    if (idx >= number_of_entries_) {
      return buffer_[0];
    }
    return buffer_[idx];
  }

  /**
   * returns a constant entry.
   * \param i index_ of entry counting from last added (last=0,...)
   * \return a reference to the buffer_ entry
   */
  inline const V& operator[](int i) const {
    assert(!empty());
    auto idx = (N + current_ - i) % N;
    if (idx >= number_of_entries_) {
      return buffer_[0];
    }
    return buffer_[idx];
  }

  /** Returns the number of elements that are currently in the ring buffer_
   * \return The number
   */
  inline int getNumberOfEntries() const { return number_of_entries_; }

  /**
   * Returns the maximum entry count.
   * \return The maximum entry count.
   */
  inline int getMaxEntries() const { return N; }

  /**
   * Determines whether maximum entry count equals actual number of entries.
   * @return true iff getMaxEntries == getNumberOfEntries.
   */
  inline bool isFilled() const { return getMaxEntries() == getNumberOfEntries(); }

  /**
   * Determines whether the buffer_ is empty.
   * \return True, if the number of entries is 0.
   */
  inline bool isEmpty() const { return !static_cast<bool>(number_of_entries_); }

  bool full() const { return number_of_entries_ == N; }

  /** The number of elements currently stored in the buffer_. */
  std::size_t size() const { return number_of_entries_; }
  inline bool empty() const { return isEmpty(); }

  /** Access the the last element of the buffer_. */
  V& back() {
    assert(!empty());
    return (*this)[0];
  }
  const V& back() const {
    assert(!empty());
    return (*this)[0];
  }

private:
  int current_;
  int number_of_entries_;
  V buffer_[N];
};
