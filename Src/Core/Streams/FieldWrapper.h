/**
 * @file FieldWrapper.h
 *
 * Field reference wrappers to use to bridge with ROS2
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

// Inspired by https://en.cppreference.com/w/cpp/utility/functional/reference_wrapper.

#include <array>
#include <utility>
#include <type_traits>
#include <functional>
#include <cstdint>
#include <string>

#include "InOut.h"
#include "Core/Streams/Streamable.h"
#include "Core/System/BHAssert.h"

#ifdef ENABLE_ROS
#define FIELD_WRAPPER(type, init_bh, init_ros, name) (FieldWrapperROS<type>)(init_ros) name
#define FIELD_WRAPPER_DEFAULT(type, init_ros, name) (FieldWrapperROS<type>)(init_ros) name
#else
#define FIELD_WRAPPER(type, init_bh, init_ros, name) (type)(static_cast<type>(init_bh)) name
#define FIELD_WRAPPER_DEFAULT(type, init_ros, name) (type) name
#endif

#define FIELD_WRAPPER_LEGACY(type, init_bh, init_ros, name) (type)(static_cast<type>(init_bh)) name
#define FIELD_WRAPPER_DEFAULT_LEGACY(type, init_ros, name) (type) name

namespace internal {
  template <class T> constexpr T& FUN(T& t) noexcept { return t; }
  template <class T> constexpr T& FUN(const T& t) noexcept { return const_cast<T&>(t); }
  template <class T> void FUN(T&&) = delete;
  template <class T> struct remove_cvref { using type = std::remove_cv_t<std::remove_reference_t<T>>; };

  template <typename T> struct is_string { static const bool value = false; };
  template <typename T, class Traits, class Alloc> struct is_string<std::basic_string<T, Traits, Alloc>> {
    static const bool value = true;
  };
} // namespace internal

template <typename T, typename = void> class FieldWrapperROS;

// Wrapper for integral floating point and string types
template <typename T>
class FieldWrapperROS<
  T,
  std::enable_if_t<std::is_integral<T>::value || std::is_floating_point<T>::value || internal::is_string<T>::value>> {
public:
  using wrapped_type = T;

  template <typename U,
            typename = decltype(internal::FUN<T>(std::declval<U>()),
                                std::enable_if_t<!std::is_same<FieldWrapperROS, internal::remove_cvref<U>>::value>())>
  FieldWrapperROS(U&& u) : ptr_(std::addressof(internal::FUN<T>(std::forward<U>(u)))) {}
  template <typename U> FieldWrapperROS(U&& u, internal::ref_init_tag tag) : FieldWrapperROS(std::forward<U>(u)) {}
  template <typename U> FieldWrapperROS(U&& u, internal::ref_alias_tag tag) : FieldWrapperROS(std::forward<U>(u)) {}

  FieldWrapperROS(const FieldWrapperROS&) noexcept = delete;

  FieldWrapperROS& operator=(const FieldWrapperROS& x) noexcept {
    ASSERT(ptr_ != nullptr);
    *ptr_ = *x.ptr_;
    return *this;
  }
  FieldWrapperROS& operator=(const T& v) {
    ASSERT(ptr_ != nullptr);
    *ptr_ = v;
    return *this;
  }

  operator T&() noexcept {
    ASSERT(ptr_ != nullptr);
    return *ptr_;
  }
  operator const T&() const noexcept {
    ASSERT(ptr_ != nullptr);
    return *ptr_;
  }
  T& get() noexcept {
    ASSERT(ptr_ != nullptr);
    return *ptr_;
  }

private:
  FieldWrapperROS() {}

  T* ptr_{nullptr};
};

template <typename T> class FieldWrapperROS<T, std::enable_if_t<std::is_enum<T>::value>> {
public:
  using wrapped_type = T;
  using enum_type = T;

  FieldWrapperROS(uint8_t& ref) : ptr_(static_cast<uint8_t*>(&ref)) { *ptr_ = 0; }
  FieldWrapperROS(const uint8_t& ref) : ptr_(const_cast<uint8_t*>(static_cast<const uint8_t*>(&ref))) { *ptr_ = 0; }
  template <typename U> FieldWrapperROS(U&& u, internal::ref_init_tag tag) : FieldWrapperROS(std::forward<U>(u)) {}
  template <typename U> FieldWrapperROS(U&& u, internal::ref_alias_tag tag) : FieldWrapperROS(std::forward<U>(u)) {}

  FieldWrapperROS(const FieldWrapperROS&) noexcept = delete;

  FieldWrapperROS& operator=(const FieldWrapperROS& x) noexcept {
    ASSERT(ptr_ != nullptr);
    *ptr_ = *x.ptr_;
    return *this;
  }
  FieldWrapperROS& operator=(const T& v) {
    ASSERT(ptr_ != nullptr);
    *ptr_ = v;
    return *this;
  }

  operator T&() noexcept {
    ASSERT(ptr_ != nullptr);
    return *static_cast<T*>(static_cast<void*>(ptr_));
  }
  operator const T&() const noexcept {
    ASSERT(ptr_ != nullptr);
    return *static_cast<T*>(static_cast<void*>(ptr_));
  }
  uint8_t& get() noexcept {
    ASSERT(ptr_ != nullptr);
    return *ptr_;
  }

private:
  FieldWrapperROS() {}

  uint8_t* ptr_{nullptr};
};

namespace internal {
  template <typename T> struct is_enum<T, std::void_t<typename T::enum_type>> : public std::true_type {};
} // namespace internal

namespace Streaming {
  template <typename T> struct Function<FieldWrapperROS<T>> {
    inline static const char* (*cast(const char* (*enumToString)(T)))(unsigned char) {
      return (const char* (*)(unsigned char))enumToString;
    }
  };
} // namespace Streaming

namespace internal {
  template <typename T, typename = void> class array_value_type {
  public:
    using type = std::remove_extent_t<T>;
  };
  template <typename T>
  class array_value_type<T, std::enable_if_t<internal::is_enum<typename std::remove_extent_t<T>>::value>> {
  public:
    using type = uint8_t;
  };
} // namespace internal

template <typename T>
class FieldWrapperROS<T,
                      std::enable_if_t<std::rank<T>::value == 1 &&
                                       (std::is_integral<typename internal::array_value_type<T>::type>::value ||
                                        std::is_floating_point<typename internal::array_value_type<T>::type>::value)>> {
public:
  using wrapped_type = T;
  using value_type = typename internal::array_value_type<T>::type;
  using real_type = std::remove_extent_t<T>;
  static constexpr int array_len = std::extent<T>::value;
  using ref_type = real_type (&)[array_len];

  template <typename U,
            typename = decltype(internal::FUN<std::array<value_type, array_len>>(std::declval<U>()),
                                std::enable_if_t<!std::is_same<FieldWrapperROS, internal::remove_cvref<U>>::value>())>
  FieldWrapperROS(U&& u) : ptr_(std::addressof(internal::FUN<std::array<value_type, array_len>>(std::forward<U>(u)))) {}
  template <typename U> FieldWrapperROS(U&& u, internal::ref_init_tag tag) : FieldWrapperROS(std::forward<U>(u)) {}
  template <typename U> FieldWrapperROS(U&& u, internal::ref_alias_tag tag) : FieldWrapperROS(std::forward<U>(u)) {}

  FieldWrapperROS(const FieldWrapperROS&) noexcept = delete;

  real_type& operator[](int i) { return *static_cast<real_type*>(static_cast<void*>(&ptr_->at(i))); }
  const real_type& operator[](int i) const { return *static_cast<real_type*>(static_cast<void*>(&ptr_->at(i))); }

  FieldWrapperROS& operator=(const FieldWrapperROS& x) noexcept {
    ASSERT(ptr_ != nullptr);
    for (int i = 0; i < array_len; ++i) {
      ptr_->at(i) = x.ptr_->at(i);
    }
    return *this;
  }

  operator ref_type() noexcept {
    ASSERT(ptr_ != nullptr);
    return *static_cast<real_type(*)[array_len]>(static_cast<void*>(ptr_));
  }
  operator ref_type() const noexcept {
    ASSERT(ptr_ != nullptr);
    return *static_cast<real_type(*)[array_len]>(static_cast<void*>(ptr_));
  }
  std::array<value_type, array_len>& get() noexcept {
    ASSERT(ptr_ != nullptr);
    return *ptr_;
  }

private:
  FieldWrapperROS() {}

  std::array<value_type, array_len>* ptr_;
};

template <typename T>
class FieldWrapperROS<T,
                      std::enable_if_t<std::rank<T>::value == 1, std::void_t<typename std::remove_extent_t<T>::ROSType>>> {
public:
  using wrapped_type = T;
  using value_type = typename internal::array_value_type<T>::type::ROSType;
  using real_type = std::remove_extent_t<T>;
  static constexpr int array_len = std::extent<T>::value;
  using ref_type = real_type (&)[array_len];

  template <typename U,
            typename = decltype(internal::FUN<std::array<value_type, array_len>>(std::declval<U>()),
                                std::enable_if_t<!std::is_same<FieldWrapperROS, internal::remove_cvref<U>>::value>())>
  FieldWrapperROS(U&& u, bool init = true)
      : ptr_(std::addressof(internal::FUN<std::array<value_type, array_len>>(std::forward<U>(u)))) {
    init_vec(init);
  }
  template <typename U> FieldWrapperROS(U&& u, internal::ref_init_tag tag) : FieldWrapperROS(std::forward<U>(u), true) {}
  template <typename U> FieldWrapperROS(U&& u, internal::ref_alias_tag tag) : FieldWrapperROS(std::forward<U>(u), false) {}

  FieldWrapperROS(const FieldWrapperROS&) noexcept = delete;

  real_type& operator[](int i) { return vec_.at(i); }
  const real_type& operator[](int i) const { return vec_.at(i); }

  FieldWrapperROS& operator=(const FieldWrapperROS& x) noexcept {
    ASSERT(ptr_ != nullptr);
    for (int i = 0; i < array_len; ++i) {
      ptr_->at(i) = x.ptr_->at(i);
    }
    init_vec();
    return *this;
  }

  operator ref_type() noexcept { return *static_cast<real_type(*)[array_len]>(static_cast<void*>(vec_.data())); }
  operator ref_type() const noexcept { return *static_cast<real_type(*)[array_len]>(static_cast<void*>(vec_.data())); }
  real_type* get() noexcept { return vec_.data(); }

private:
  FieldWrapperROS() {}

  void init_vec(bool init = false) {
    vec_.clear();
    for (int i = 0; i < array_len; ++i) {
      if (init) {
        vec_.emplace_back(ptr_->at(i), internal::ref_init_tag{});
      } else {
        vec_.emplace_back(ptr_->at(i), internal::ref_alias_tag{});
      }
    }
  }

  std::array<value_type, array_len>* ptr_;
  std::vector<real_type> vec_;
};

template <typename T> class FieldWrapperROS<T, std::void_t<typename T::ROSType>> : public T {
public:
  using wrapped_type = T;

  template <typename U, typename = std::enable_if_t<!std::is_same<FieldWrapperROS, internal::remove_cvref<U>>::value>()>
  FieldWrapperROS(U&& u) : T(std::forward<U>(u), internal::ref_init_tag()) {}
  template <typename U> FieldWrapperROS(U&& u, internal::ref_init_tag tag) : T(std::forward<U>(u), tag) {}
  template <typename U> FieldWrapperROS(U&& u, internal::ref_alias_tag tag) : T(std::forward<U>(u), tag) {}

  FieldWrapperROS(const FieldWrapperROS&) noexcept = delete;

  FieldWrapperROS& operator=(const T& u) {
    T::operator=(u);
    return *this;
  }

  operator T&() noexcept { return *this; }
  operator const T&() const noexcept { return *this; }
  T& get() noexcept { return *this; }
};

template <typename T>
class FieldWrapperROS<
  std::vector<T>,
  std::enable_if_t<std::is_integral<T>::value || std::is_floating_point<T>::value || internal::is_string<T>::value>> {
public:
  using wrapped_type = std::vector<T>;

  template <typename U,
            typename = decltype(internal::FUN<std::vector<T>>(std::declval<U>()),
                                std::enable_if_t<!std::is_same<FieldWrapperROS, internal::remove_cvref<U>>::value>())>
  FieldWrapperROS(U&& u) : ptr_(std::addressof(internal::FUN<std::vector<T>>(std::forward<U>(u)))) {}
  template <typename U> FieldWrapperROS(U&& u, internal::ref_init_tag tag) : FieldWrapperROS(std::forward<U>(u)) {}
  template <typename U> FieldWrapperROS(U&& u, internal::ref_alias_tag tag) : FieldWrapperROS(std::forward<U>(u)) {}

  FieldWrapperROS(const FieldWrapperROS&) noexcept = delete;

  FieldWrapperROS& operator=(const FieldWrapperROS& x) noexcept {
    if (&x == this) {
      return *this;
    }
    ASSERT(ptr_ != nullptr);
    ptr_->clear();
    for (const auto& el : *x.ptr_) {
      push_back(el);
    }
    return *this;
  }

  // Minimum vector operations typically required.
  T& operator[](size_t pos) { return ptr_->at(pos); }
  const T& operator[](size_t pos) const { return ptr_->at(pos); }

  T& at(size_t pos) { return ptr_->at(pos); }
  const T& at(size_t pos) const { return ptr_->at(pos); }

  typename wrapped_type::iterator begin() { return ptr_->begin(); }
  typename wrapped_type::iterator end() { return ptr_->end(); }
  typename wrapped_type::const_iterator begin() const { return ptr_->begin(); }
  typename wrapped_type::const_iterator end() const { return ptr_->end(); }

  bool empty() const noexcept { return ptr_->empty(); }
  size_t size() const noexcept { return ptr_->size(); }
  void clear() { ptr_->clear(); }
  const T& front() const noexcept {
    ASSERT(ptr_ != nullptr && !ptr_->empty());
    return ptr_->front();
  }
  const T& back() const noexcept {
    ASSERT(ptr_ != nullptr && !ptr_->empty());
    return ptr_->back();
  }
  T* data() noexcept { return ptr_->data(); }

  void push_back(const T& value) { ptr_->push_back(value); }
  void resize(size_t n) { ptr_->resize(n); }

  // clang-format off
  operator std::vector<T>&() noexcept {
    ASSERT(ptr_ != nullptr);
    return *ptr_;
  }
  operator const std::vector<T>&() const noexcept {
    ASSERT(ptr_ != nullptr);
    return *ptr_;
  }
  // clang-format on
  std::vector<T>& get() noexcept {
    ASSERT(ptr_ != nullptr);
    return *ptr_;
  }

private:
  std::vector<T>* ptr_;
};

namespace Streaming {
  template <typename E> struct Streamer<FieldWrapperROS<std::vector<E>, std::void_t<typename E::ROSType>>> {
    typedef FieldWrapperROS<std::vector<E>> S;
    static void stream(In* in, Out* out, const char* name, S& s, const char* (*enumToString)(unsigned char)) {
#ifndef RELEASE
      registerDefaultElement(static_cast<typename S::internal_wrapped_type>(s));
      registerWithSpecification(name, typeid(&s[0]));
      if (enumToString)
        Streaming::registerEnum(typeid(s[0]), (const char* (*)(unsigned char))enumToString);
#endif
      if (in) {
        in->select(name, -1);
        unsigned _size;
        *in >> _size;
        s.resize(_size);
        if (!s.empty())
          streamStaticArray(*in, s.data(), s.size() * sizeof(s[0]), enumToString);
        in->deselect();
      } else {
        out->select(name, -1);
        *out << (unsigned)s.size();
        if (!s.empty())
          streamStaticArray(*out, s.data(), s.size() * sizeof(s[0]), enumToString);
        out->deselect();
      }
    }
  };
} // namespace Streaming

template <typename T> class FieldWrapperROS<std::vector<T>, std::void_t<typename T::ROSType>> {
public:
  using internal_wrapped_type = std::vector<T>;

  template <typename U,
            typename = decltype(internal::FUN<std::vector<typename T::ROSType>>(std::declval<U>()),
                                std::enable_if_t<!std::is_same<FieldWrapperROS, internal::remove_cvref<U>>::value>())>
  FieldWrapperROS(U&& u, bool init = true)
      : ptr_(std::addressof(internal::FUN<std::vector<typename T::ROSType>>(std::forward<U>(u)))) {
    init_vec(init);
  }
  template <typename U> FieldWrapperROS(U&& u, internal::ref_init_tag tag) : FieldWrapperROS(std::forward<U>(u), true) {}
  template <typename U> FieldWrapperROS(U&& u, internal::ref_alias_tag tag) : FieldWrapperROS(std::forward<U>(u), false) {}

  FieldWrapperROS(const FieldWrapperROS&) noexcept = delete;

  FieldWrapperROS& operator=(const FieldWrapperROS& x) noexcept {
    if (&x == this) {
      return *this;
    }
    ASSERT(ptr_ != nullptr);
    vec_.clear();
    ptr_->clear();
    for (const auto& el : *x.ptr_) {
      push_back(el);
    }
    return *this;
  }

  // Minimum vector operations typically required.
  T& operator[](size_t pos) {
    ASSERT(vec_.size() == ptr_->size());
    return vec_.at(pos);
  }
  const T& operator[](size_t pos) const {
    ASSERT(vec_.size() == ptr_->size());
    return vec_.at(pos);
  }

  T& at(size_t pos) { return vec_.at(pos); }
  const T& at(size_t pos) const { return vec_.at(pos); }

  typename std::vector<T>::iterator begin() { return vec_.begin(); }
  typename std::vector<T>::iterator end() { return vec_.end(); }
  typename std::vector<T>::const_iterator begin() const { return vec_.begin(); }
  typename std::vector<T>::const_iterator end() const { return vec_.end(); }

  bool empty() const noexcept { return vec_.empty(); }
  size_t size() const noexcept { return vec_.size(); }
  const T& front() const noexcept { return vec_.front(); }
  const T& back() const noexcept { return vec_.back(); }
  T* data() noexcept { return vec_.data(); }

  // Make a copy to ensure the references are bound directly to the base class.
  void push_back(T value) { push_back(static_cast<typename T::ROSType>(value)); }
  void resize(size_t n) {
    ptr_->resize(n);
    init_vec();
  }
  void reserve(size_t n) {
    ptr_->reserve(n);
    init_vec();
  }
  void clear() {
    ptr_->clear();
    init_vec();
  }

  // clang-format off
  operator const std::vector<T>&() const noexcept { return vec_; }
  // clang-format on
  std::vector<T>& get() noexcept { return vec_; }

private:
  void push_back(typename T::ROSType value) {
    ASSERT(vec_.size() == ptr_->size());
    ASSERT(ptr_->size() <= ptr_->capacity());
    bool at_capacity = (ptr_->size() == ptr_->capacity());
    ptr_->push_back(value);
    if (!at_capacity) {
      vec_.emplace_back(ptr_->back(), internal::ref_alias_tag{});
    } else {
      init_vec();
    }
  }

  void init_vec(bool init = false) {
    vec_.clear();
    for (auto& el : *ptr_) {
      if (init) {
        vec_.emplace_back(el, internal::ref_init_tag{});
      } else {
        vec_.emplace_back(el, internal::ref_alias_tag{});
      }
    }
  }

  std::vector<T> vec_;
  std::vector<typename T::ROSType>* ptr_;
};

// deduction guides
#ifdef ENABLE_ROS
template <class T> FieldWrapperROS(T&) -> FieldWrapperROS<T>;
#endif
