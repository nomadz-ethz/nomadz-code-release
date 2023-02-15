/**
 * @file RingBufferNew.h
 *
 * The file declares a ring buffer. The type of the elements must be assignable and
 * copyable. They do not need to provide a default constuctor. The interface of the
 * class is similar to types of the standard template library and it also supports
 * for-each loops.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Thomas Röfer Thomas Röfer
 */

#pragma once

#include <cstddef>
#include <cstring>
#include "Core/System/BHAssert.h"
#include "Core/System/SystemCall.h"

template <typename T, std::size_t n = 0> class RingBufferNew {
private:
  T* buffer;               /**< Stores the elements of the buffer. */
  std::size_t allocated;   /**< The capacity of the buffer. */
  std::size_t head = 0;    /**< The next entry that will be used for push_front(). */
  std::size_t entries = 0; /**< The number of entries in the buffer. */

public:
  /** A class for iterators with its typical interface. */
  class iterator {
  private:
    RingBufferNew<T, n>& buffer; /**< The buffer. */
    std::size_t index;           /**< Index of the current entry. */

  public:
    iterator(RingBufferNew<T, n>& buffer, std::size_t index) : buffer(buffer), index(index) {}
    T* operator->() const { return &buffer[index]; }
    T& operator*() const { return buffer[index]; }
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
    const RingBufferNew<T, n>& buffer; /**< The buffer. */
    std::size_t index;                 /**< Index of the current entry. */

  public:
    const_iterator(const RingBufferNew<T, n>& buffer, std::size_t index) : buffer(buffer), index(index) {}
    const T* operator->() const { return &buffer[index]; }
    const T& operator*() const { return buffer[index]; }
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

  /**
   * Constructor.
   * @param capacity The maximum number of entries the buffer can store. If not specified,
   *                 the second template parameter is used as default capacity.
   */
  RingBufferNew(size_t capacity = n)
      : buffer(reinterpret_cast<T*>(SystemCall::alignedMalloc(capacity * sizeof(T)))), allocated(capacity) {}

  /**
   * Copy constructor.
   * @param other The buffer this one is constructed from.
   */
  RingBufferNew(const RingBufferNew& other)
      : buffer(reinterpret_cast<T*>(SystemCall::alignedMalloc(other.allocated * sizeof(T)))), allocated(other.allocated) {
    for (std::size_t i = other.entries; i-- > 0;)
      push_front(other[i]);
  }

  /** Destructor. */
  ~RingBufferNew() {
    clear();
    if (buffer)
      SystemCall::alignedFree(reinterpret_cast<char*>(buffer));
  }

  /**
   * Assignment operator.
   * @param other The buffer that is assigned to this one.
   */
  RingBufferNew& operator=(const RingBufferNew& other) {
    clear();
    if (allocated != other.allocated) {
      if (buffer)
        SystemCall::alignedFree(reinterpret_cast<char*>(buffer));
      buffer = reinterpret_cast<T*>(SystemCall::alignedMalloc(other.allocated * sizeof(T)));
    }
    allocated = other.allocated;
    head = 0;
    entries = 0;
    for (std::size_t i = other.entries; i-- > 0;)
      push_front(other[i]);
    return *this;
  }

  /** Empties the buffer. */
  void clear() {
    while (!empty())
      pop_back();
  }

  /**
   * Adds a new entry to the front of the buffer. The new entry is accessible under
   * index 0, front(), and *begin(). If the buffer was already full, the entry at
   * back() is lost.
   * @param value The value that is added to the buffer.
   */
  void push_front(const T& value) {
    ASSERT(allocated);
    if (entries < allocated) {
      new (buffer + head) T(value);
      ++entries;
    } else
      buffer[head] = value;
    head = (head + 1) % allocated;
  }

  /** Removes the entry back() from the buffer. */
  void pop_back() {
    ASSERT(!empty());
    buffer[(allocated + head - entries) % allocated].~T();
    --entries;
  }

  /**
   * Access to the individual entries of the buffer.
   * @param index The index of the element. Element 0 is the same as front(), element
   *              size()-1 is the same as back().
   */
  T& operator[](size_t index) {
    ASSERT(!empty());
    auto idx = (allocated + head - index - 1) % allocated;
    if (idx >= entries) {
      return buffer[0];
    }
    return buffer[idx];
  }
  const T& operator[](size_t index) const {
    ASSERT(!empty());
    auto idx = (allocated + head - index - 1) % allocated;
    if (idx >= entries) {
      return buffer[0];
    }
    return buffer[idx];
  }

  /** Access the the first element of the buffer. */
  T& front() {
    ASSERT(!empty());
    return (*this)[0];
  }
  const T& front() const {
    ASSERT(!empty());
    return (*this)[0];
  }

  /** Access the the last element of the buffer. */
  T& back() {
    ASSERT(!empty());
    return (*this)[entries - 1];
  }
  const T& back() const {
    ASSERT(!empty());
    return (*this)[entries - 1];
  }

  /** The number of elements currently stored in the buffer. */
  std::size_t size() const { return entries; }

  /** The maximum number of elements that can be stored in the buffer. */
  size_t capacity() const { return allocated; }

  /**
   * Changes the capacity of the buffer. If it actually changes, the complexity is O(size()).
   * @param capacity The maximum number of entries the buffer can store.
   */
  void reserve(std::size_t capacity) {
    if (capacity != allocated) {
      while (size() > capacity)
        pop_back();

      T* prev = buffer;
      buffer = reinterpret_cast<T*>(SystemCall::alignedMalloc(capacity * sizeof(T)));
      if (head >= entries)
        std::memcpy(buffer, prev + head - entries, entries * sizeof(T));
      else {
        std::memcpy(buffer, prev + allocated - (entries - head), (entries - head) * sizeof(T));
        std::memcpy(buffer + entries - head, prev, head * sizeof(T));
      }
      allocated = capacity;
      head = allocated > entries ? entries : 0;
      if (prev)
        SystemCall::alignedFree(reinterpret_cast<char*>(prev));
    }
  }

  /** Is the buffer empty? */
  bool empty() const { return entries == 0; }

  /** Is the buffer full, i.e. will the next push_front() drop the element back()? */
  bool full() const { return entries == allocated; }

  /** Returns an iterator pointing at the front() of the buffer. */
  iterator begin() { return iterator(*this, 0); }
  const_iterator begin() const { return const_iterator(*this, 0); }

  /** Returns an iterator pointing at behind the back() of the buffer. */
  iterator end() { return iterator(*this, entries); }
  const_iterator end() const { return const_iterator(*this, entries); }

protected:
  /** Is the buffer at the begin of a cycle? */
  bool cycled() const { return head == 0; }
};