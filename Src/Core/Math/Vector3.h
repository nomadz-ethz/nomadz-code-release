/**
 * @file Vector3.h
 *
 * Contains template class Vector3 of type V
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * and Max Risler
 */

#pragma once

#include "Common.h"
#include "Core/Streams/AutoStreamable.h"
#include "Vector2.h"

namespace internal {
  template <typename T> class vector3_ros_wrapper : public ROSMessage<void, false> {
  public:
    using base_type = vector3_ros_wrapper;
    T x{}, y{}, z{};
  };
} // namespace internal

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/vector3f.hpp"
#include "nomadz_msgs/msg/vector3i.hpp"
namespace internal {
  template <> class vector3_ros_wrapper<float> : public ROSMessage<nomadz_msgs::msg::Vector3f> {
  public:
    using base_type = nomadz_msgs::msg::Vector3f;
  };
  template <> class vector3_ros_wrapper<int> : public ROSMessage<nomadz_msgs::msg::Vector3i> {
  public:
    using base_type = nomadz_msgs::msg::Vector3i;
  };
} // namespace internal
#endif

/** This class represents a 3-vector */
template <class V = float> class Vector3 : public Streamable, public internal::vector3_ros_wrapper<V> {
public:
  virtual void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(x);
    STREAM(y);
    STREAM(z);
    STREAM_REGISTER_FINISH;
  }

  /** The vector values */
  V &x, &y, &z;

  /** Default constructor. */
  inline Vector3()
      : x(internal::vector3_ros_wrapper<V>::base_type::x), y(internal::vector3_ros_wrapper<V>::base_type::y),
        z(internal::vector3_ros_wrapper<V>::base_type::z) {
    x = V();
    y = V();
    z = V();
  }
  template <typename U> inline Vector3(U& ref, internal::ref_init_tag) : x(ref.x), y(ref.y), z(ref.z) {
    x = V();
    y = V();
    z = V();
  }
  template <typename U> inline Vector3(U& ref, internal::ref_alias_tag) : x(ref.x), y(ref.y), z(ref.z) {}

  /** Constructor. */
  inline Vector3(V xn, V yn, V zn) : Vector3() {
    x = xn;
    y = yn;
    z = zn;
  }

  /** Copy constructor
   *\param other The other vector that is copied to this one
   */
  inline Vector3(const Vector3& other) : Vector3() {
    x = other.x;
    y = other.y;
    z = other.z;
  }

  inline Vector3(const Vector2<>& other) : Vector3() {
    x = other.x;
    y = other.y;
    z = 0.f;
  }

  /** Copy constructor from different element type
   *\param O The type of other's elements
   *\param other The other vector that is copied to this one
   */
  template <typename O> inline explicit Vector3(const Vector3<O>& other) : Vector3() {
    x = (V)other.x;
    y = (V)other.y;
    z = (V)other.z;
  }

  /** Assignment operator
   *\param other The other vector that is assigned to this one
   *\return A reference to this object after the assignment.
   */
  inline Vector3& operator=(const Vector3& other) {
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
  }

  /** Addition of another vector to this one.
   *\param other The other vector that will be added to this one
   *\return A reference to this object after the calculation.
   */
  inline Vector3& operator+=(const Vector3& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  /** Substraction of this vector from another one.
   *\param other The other vector this one will be substracted from
   *\return A reference to this object after the calculation.
   */
  inline Vector3& operator-=(const Vector3& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }

  /** Multiplication of this vector by a factor.
   *\param factor The factor this vector is multiplied by
   *\return A reference to this object after the calculation.
   */
  inline Vector3& operator*=(const V& factor) {
    x *= factor;
    y *= factor;
    z *= factor;
    return *this;
  }

  /** Division of this vector by a factor.
   *\param factor The factor this vector is divided by
   *\return A reference to this object after the calculation.
   */
  inline Vector3& operator/=(const V& factor) {
    if (factor == V())
      return *this;
    x /= factor;
    y /= factor;
    z /= factor;
    return *this;
  }

  /** Addition of another vector to this one.
   *\param other The other vector that will be added to this one
   *\return A new object that contains the result of the calculation.
   */
  inline Vector3 operator+(const Vector3& other) const { return Vector3(*this) += other; }

  /** Subtraction of another vector to this one.
   *\param other The other vector that will be added to this one
   *\return A new object that contains the result of the calculation.
   */
  inline Vector3 operator-(const Vector3& other) const { return Vector3(*this) -= other; }

  /** Negation of this vector.
   *\return A new object that contains the result of the calculation.
   */
  inline Vector3 operator-() const { return Vector3(-x, -y, -z); }

  /** Inner product of this vector and another one.
   *\param other The other vector this one will be multiplied by
   *\return The inner product.
   */
  inline V operator*(const Vector3& other) const { return x * other.x + y * other.y + z * other.z; }

  /** Multiplication of this vector by a factor.
   *\param factor The factor this vector is multiplied by
   *\return A new object that contains the result of the calculation.
   */
  inline Vector3 operator*(const V& factor) const { return Vector3(*this) *= factor; }

  /** Division of this vector by a factor.
   *
   *\param factor The factor this vector is divided by
   *\return A new object that contains the result of the calculation.
   */
  inline Vector3 operator/(const V& factor) const { return Vector3(*this) /= factor; }

  /** Comparison of another vector with this one.
   *\param other The other vector that will be compared to this one
   *\return Whether the two vectors are equal.
   */
  inline bool operator==(const Vector3& other) const { return x == other.x && y == other.y && z == other.z; }

  /** Comparison of another vector with this one.
   *\param other The other vector that will be compared to this one
   *\return Whether the two vectors are unequal.
   */
  inline bool operator!=(const Vector3& other) const { return x != other.x || y != other.y || z != other.z; }

  /**
   * array-like member access.
   * \param i index of coordinate
   * \return reference to x, y or z
   */
  inline V& operator[](int i) { return const_cast<V&>(static_cast<const Vector3&>(*this).operator[](i)); }

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
    if (i == 2) {
      return z;
    }
    std::abort();
  }

  /** Calculation of the length of this vector.
   *\return The length.
   */
  inline V abs() const { return (V)sqrt(float((x * x) + (y * y) + (z * z))); }

  /** Calculation of the square length of this vector.
   *\return length*length.
   */
  inline V squareAbs() const { return x * x + y * y + z * z; }

  /** Crossproduct of this vector and another vector.
   *\param other The factor this vector is multiplied with.
   *\return A new object that contains the result of the calculation.
   */
  inline Vector3 operator^(const Vector3& other) const {
    return Vector3(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
  }

  /** Crossproduct of this vector and another vector.
   *\param other The factor this vector is multiplied with.
   *\return A reference to this object after the calculation.
   */
  inline Vector3& operator^=(const Vector3& other) { return *this = *this ^ other; }

  /** normalize this vector.
   *\param len The length, the vector should be normalized to, default=1.
   *\return the normalized vector.
   */
  Vector3& normalize(V len) {
    const V length = abs();
    if (length == V())
      return *this;
    *this *= len;
    return *this /= length;
  }

  /** normalize this vector.
   *\return the normalized vector.
   */
  Vector3& normalize() { return *this /= abs(); }

  /** Return the cross product **/
  Vector3 cross(const Vector3& v2) {
    Vector3 vr;
    vr.x = -this->z * v2.y + this->y * v2.z;
    vr.y = this->z * v2.x - this->x * v2.z;
    vr.z = -this->y * v2.x + this->x * v2.y;
    return vr;
  }
  Vector2<> toVec2() { return Vector2<>(this->x, this->y); }
};
