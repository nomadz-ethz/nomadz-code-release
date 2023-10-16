/**
 * @file Vector2.h
 *
 * Contains template class Vector2 of type V
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * and Max Risler
 */

#pragma once

#include "Common.h"
#include "Core/Streams/AutoStreamable.h"

namespace internal {
  template <typename T> class vector2_ros_wrapper : public ROSMessage<void, false> {
  public:
    using base_type = vector2_ros_wrapper;
    T x{}, y{};
  };
} // namespace internal

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/vector2f.hpp"
#include "nomadz_msgs/msg/vector2i.hpp"
namespace internal {
  template <> class vector2_ros_wrapper<float> : public ROSMessage<nomadz_msgs::msg::Vector2f> {
  public:
    using base_type = nomadz_msgs::msg::Vector2f;
  };
  template <> class vector2_ros_wrapper<int> : public ROSMessage<nomadz_msgs::msg::Vector2i> {
  public:
    using base_type = nomadz_msgs::msg::Vector2i;
  };
} // namespace internal
#endif

/** This class represents a 2-vector */
template <class V = float> class Vector2 : public Streamable, public internal::vector2_ros_wrapper<V> {
public:
  virtual void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(x);
    STREAM(y);
    STREAM_REGISTER_FINISH;
  }

  virtual ~Vector2() {}

  /** The vector values */
  V &x, &y;

  /** Default constructor. */
  inline Vector2() : x(internal::vector2_ros_wrapper<V>::base_type::x), y(internal::vector2_ros_wrapper<V>::base_type::y) {
    x = V();
    y = V();
  }
  template <typename U> inline Vector2(U& ref, internal::ref_init_tag) : x(ref.x), y(ref.y) {
    x = V();
    y = V();
  }
  template <typename U> inline Vector2(U& ref, internal::ref_alias_tag) : x(ref.x), y(ref.y) {}

  /** Constructor. */
  inline Vector2(V xn, V yn) : Vector2() {
    x = xn;
    y = yn;
  }

  /** Copy constructor
   *\param other The other vector that is copied to this one
   */
  inline Vector2(const Vector2& other) : Vector2() {
    x = other.x;
    y = other.y;
  }

  /** Copy constructor from different element type
   *\param O The type of other's elements
   *\param other The other vector that is copied to this one
   */
  template <typename O> inline explicit Vector2(const Vector2<O>& other) : Vector2() {
    x = (V)other.x;
    y = (V)other.y;
  }

  /** Assignment operator
   *\param other The other vector that is assigned to this one
   *\return A reference to this object after the assignment.
   */
  inline Vector2& operator=(const Vector2& other) {
    x = other.x;
    y = other.y;
    return *this;
  }

  /** Addition of another vector to this one.
   *\param other The other vector that will be added to this one
   *\return A reference to this object after the calculation.
   */
  inline Vector2& operator+=(const Vector2& other) {
    x += other.x;
    y += other.y;
    return *this;
  }

  /** Substraction of this vector from another one.
   *\param other The other vector this one will be substracted from
   *\return A reference to this object after the calculation.
   */
  inline Vector2& operator-=(const Vector2& other) {
    x -= other.x;
    y -= other.y;
    return *this;
  }

  /** Multiplication of this vector by a factor.
   *\param factor The factor this vector is multiplied by
   *\return A reference to this object after the calculation.
   */
  inline Vector2& operator*=(const V& factor) {
    x *= factor;
    y *= factor;
    return *this;
  }

  /** Division of this vector by a factor.
   *\param factor The factor this vector is divided by
   *\return A reference to this object after the calculation.
   */
  inline Vector2& operator/=(const V& factor) {
    if (factor == V())
      return *this;
    x /= factor;
    y /= factor;
    return *this;
  }

  /** Addition of another vector to this one.
   *\param other The other vector that will be added to this one
   *\return A new object that contains the result of the calculation.
   */
  inline Vector2 operator+(const Vector2& other) const { return Vector2(*this) += other; }

  /** Subtraction of another vector to this one.
   *\param other The other vector that will be added to this one
   *\return A new object that contains the result of the calculation.
   */
  inline Vector2 operator-(const Vector2& other) const { return Vector2(*this) -= other; }

  /** Negation of this vector.
   *\return A new object that contains the result of the calculation.
   */
  inline Vector2 operator-() const { return Vector2(-x, -y); }

  /** Inner product of this vector and another one.
   *\param other The other vector this one will be multiplied by
   *\return The inner product.
   */
  inline V operator*(const Vector2& other) const { return x * other.x + y * other.y; }

  /** Multiplication of this vector by a factor.
   *\param factor The factor this vector is multiplied by
   *\return A new object that contains the result of the calculation.
   */
  inline Vector2 operator*(const V& factor) const { return Vector2(*this) *= factor; }

  /** Division of this vector by a factor.
   *
   *\param factor The factor this vector is divided by
   *\return A new object that contains the result of the calculation.
   */
  inline Vector2 operator/(const V& factor) const { return Vector2(*this) /= factor; }

  /** Comparison of another vector with this one.
   *\param other The other vector that will be compared to this one
   *\return Whether the two vectors are equal.
   */
  inline bool operator==(const Vector2& other) const { return x == other.x && y == other.y; }

  /** Comparison of another vector with this one.
   *\param other The other vector that will be compared to this one.
   *\return Whether the two vectors are unequal.
   */
  inline bool operator!=(const Vector2& other) const { return x != other.x || y != other.y; }

  /** Calculation of the length of this vector.
   *\return The length.
   */
  inline V abs() const { return (V)std::sqrt(((float)x) * x + ((float)y) * y); }

  /**
   * Calculation of the length of this vector
   * @return
   */
  inline float absFloat() const { return std::sqrt(((float)x) * x + ((float)y) * y); }

  /** Calculation of the square length of this vector.
   *\return length*length.
   */
  inline V squareAbs() const { return x * x + y * y; }

  inline V max() const { return std::max(std::abs(x), std::abs(y)); }

  /**
   * Calculation of the square of the vector.
   * \return The square
   */
  inline V sqr() const { return x * x + y * y; }

  /** Returns the dot product of this vector with another vector */
  inline V dot(const Vector2& other) const { return x * other.x + y * other.y; }

  /**
   * The two vectors represent the two first and second column of the
   * matrix that is created by multiplying the vector times its
   * transposed and ending up with a matrix.
   * */

  inline Vector2 VM1() const {
    Vector2<> output;
    output.x = x * x;
    output.y = x * y;
    return output;
  }

  inline Vector2 VM2() const {
    Vector2<> output;
    output.x = y * x;
    output.y = y * y;
    return output;
  }

  /** normalize this vector.
   *\param len The length, the vector should be normalized to, default=1.
   *\return the normalized vector.
   */
  Vector2& normalize(V len) {
    const V length = abs();
    if (length == V())
      return *this;
    *this *= len;
    return *this /= length;
  }

  /** normalize this vector.
   *\return the normalized vector.
   */
  Vector2& normalize() { return *this /= abs(); }

  /** the vector is rotated left by 90 degrees.
   *\return the rotated vector.
   */
  Vector2& rotateLeft() {
    V buffer = -y;
    y = x;
    x = buffer;
    return *this;
  }

  /** the vector is rotated right by 90 degrees.
   *\return the rotated vector.
   */
  Vector2& rotateRight() {
    V buffer = -x;
    x = y;
    y = buffer;
    return *this;
  }

  /** the vector is rotated by 180 degrees.
   *\return the rotated vector.
   */
  Vector2& mirror() {
    x = -x;
    y = -y;
    return *this;
  }

  /** the vector is rotated by alpha degrees.
   *\return the rotated vector.
   */
  Vector2& rotate(float alpha) {
    float buffer = (float)x;
    float a = std::cos(alpha);
    float b = std::sin(alpha);
    x = (V)(a * (float)x - b * (float)y);
    y = (V)(b * buffer + a * (float)y);
    return *this;
  }

  /**
   * array-like member access.
   * \param i index of coordinate
   * \return reference to x or y
   */
  inline V& operator[](int i) { return const_cast<V&>(static_cast<const Vector2&>(*this).operator[](i)); }

  /**
   * const array-like member access.
   * \param i index of coordinate
   * \return reference to x or y
   */
  inline const V& operator[](int i) const {
    if (i == 0) {
      return x;
    }
    if (i == 1) {
      return y;
    }
    std::abort();
  }

  /** Calculation of the angle of this vector */
  inline float angle() const { return std::atan2((float)y, (float)x); }

  /** Returns a new vector with the same direction as this one and length 1 */
  inline Vector2 direction() const { return *this / abs(); }

  /** Returns a new vector that is like this one, but rotated */
  inline Vector2 rotated(float alpha) const {
    const float c = std::cos(alpha);
    const float s = std::sin(alpha);
    return Vector2((V)(c * (float)x - s * (float)y), (V)(s * (float)x + c * (float)y));
  }

  /** Returns a new vector with a direction 90 degrees "left" of this one */
  inline Vector2 left() const { return Vector2(-y, x); }

  /** Returns a new vector with a direction 90 degrees "right" of this one */
  inline Vector2 right() const { return Vector2(y, -x); }
};
