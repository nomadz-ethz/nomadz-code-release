/**
 * @file Matrix3x3.h
 *
 * Contains template class Matrix3x3 of type V
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:Kai_Engel@gmx.de">Kai Engel</a>,
 * <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>,
 * and Colin Graf
 */

#pragma once

#include "Vector3.h"

namespace internal {
  template <typename T> class matrix3x3_ros_wrapper : public ROSMessage<void, false> {};
} // namespace internal

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/matrix3x3.hpp"
namespace internal {
  template <> class matrix3x3_ros_wrapper<float> : public ROSMessage<nomadz_msgs::msg::Matrix3x3> {
  public:
    using base_type = nomadz_msgs::msg::Matrix3x3;
  };
} // namespace internal
#endif

/**
 * This class represents a 3x3-matrix
 *
 */
template <class V = float> class Matrix3x3 : public Streamable, public internal::matrix3x3_ros_wrapper<V> {
public:
  virtual void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(c0);
    STREAM(c1);
    STREAM(c2);
    STREAM_REGISTER_FINISH;
  }

  Vector3<V> c0; /**< The first column of the matrix */
  Vector3<V> c1; /**< The second column of the matrix */
  Vector3<V> c2; /**< The third column of the matrix */

  /**
   * Default constructor.
   */
#ifdef ENABLE_ROS
  Matrix3x3()
      : c0(internal::matrix3x3_ros_wrapper<V>::base_type::c0, internal::ref_init_tag{}),
        c1(internal::matrix3x3_ros_wrapper<V>::base_type::c1, internal::ref_init_tag{}),
        c2(internal::matrix3x3_ros_wrapper<V>::base_type::c2, internal::ref_init_tag{}) {
    c0 = Vector3<V>(1, 0, 0);
    c1 = Vector3<V>(0, 1, 0);
    c2 = Vector3<V>(0, 0, 1);
  }
#else
  Matrix3x3() {
    c0 = Vector3<V>(1, 0, 0);
    c1 = Vector3<V>(0, 1, 0);
    c2 = Vector3<V>(0, 0, 1);
  }
#endif
  template <typename U> inline Matrix3x3(U& ref, internal::ref_init_tag t) : c0(ref.c0, t), c1(ref.c1, t), c2(ref.c2, t) {
    c0 = Vector3<V>(1, 0, 0);
    c1 = Vector3<V>(0, 1, 0);
    c2 = Vector3<V>(0, 0, 1);
  }
  template <typename U> inline Matrix3x3(U& ref, internal::ref_alias_tag t) : c0(ref.c0, t), c1(ref.c1, t), c2(ref.c2, t) {}

  Matrix3x3(const Vector3<V>& a) : Matrix3x3() {
    const V angle = a.abs();
    Vector3<V> axis = a;
    if (angle != V())
      axis /= angle; // normalize a, rotation is only possible with unit vectors
    const V &x = axis.x, &y = axis.y, &z = axis.z;
    // compute sine and cosine of angle because it is needed quite often for complete matrix
    const V si = (V)sin(angle), co = (V)cos(angle);
    // compute all components needed more than once for complete matrix
    const V v = 1 - co;
    const V xyv = x * y * v;
    const V xzv = x * z * v;
    const V yzv = y * z * v;
    const V xs = x * si;
    const V ys = y * si;
    const V zs = z * si;
    // compute matrix
    c0.x = x * x * v + co;
    c1.x = xyv - zs;
    c2.x = xzv + ys;
    c0.y = xyv + zs;
    c1.y = y * y * v + co;
    c2.y = yzv - xs;
    c0.z = xzv - ys;
    c1.z = yzv + xs;
    c2.z = z * z * v + co;
  }

  /**
   * Constructor.
   *
   * \param  c0  the first column of the matrix.
   * \param  c1  the second column of the matrix.
   * \param  c2  the third column of the matrix.
   */
  Matrix3x3<V>(const Vector3<V>& c0, const Vector3<V>& c1, const Vector3<V>& c2) : Matrix3x3() {
    this->c0 = c0;
    this->c1 = c1;
    this->c2 = c2;
  }

  /**
   * Copy constructor.
   *
   * \param other The other matrix that is copied to this one
   */
  Matrix3x3<V>(const Matrix3x3<V>& other) : Matrix3x3() {
    c0 = other.c0;
    c1 = other.c1;
    c2 = other.c2;
  }

  /**
   * Anti-lateral thinking constructor.
   */
  Matrix3x3<V>(const V& a11,
               const V& a12,
               const V& a13,
               const V& a21,
               const V& a22,
               const V& a23,
               const V& a31,
               const V& a32,
               const V& a33)
      : Matrix3x3() {
    c0 = Vector3<V>(a11, a21, a31);
    c1 = Vector3<V>(a12, a22, a32);
    c2 = Vector3<V>(a13, a23, a33);
  }

  /**
   * Assignment operator.
   *
   * \param  other   The other matrix that is assigned to this one
   * \return         A reference to this object after the assignment.
   */
  Matrix3x3<V>& operator=(const Matrix3x3<V>& other) {
    c0 = other.c0;
    c1 = other.c1;
    c2 = other.c2;
    return *this;
  }

  /**
   * Adds this matrix with another matrix.
   *
   * \param  other  The matrix this one is added to
   * \return         A new matrix containing the result
   *                 of the calculation.
   */
  Matrix3x3<V> operator+(const Matrix3x3<V>& other) const {
    return Matrix3x3<V>(c0 + other.c0, c1 + other.c1, c2 + other.c2);
  }
  /**
   * Adds another matrix to this matrix.
   *
   * \param  other  The other matrix that is added to this one
   * \return        A reference this object after the calculation.
   */
  Matrix3x3<V>& operator+=(const Matrix3x3<V>& other) {
    c0 += other.c0;
    c1 += other.c1;
    c2 += other.c2;
    return *this;
  }

  /**
   * Compute difference of this matrix and another one
   *
   * \param  other  The matrix which is substracted from this one
   * \return         A new matrix containing the result
   *                 of the calculation.
   */
  Matrix3x3<V> operator-(const Matrix3x3<V>& other) const {
    return Matrix3x3<V>(c0 - other.c0, c1 - other.c1, c2 - other.c2);
  }
  /**
   * Substracts another matrix from this one
   *
   * \param  other  The other matrix that is substracted from this one
   * \return        A reference this object after the calculation.
   */
  Matrix3x3<V>& operator-=(const Matrix3x3<V>& other) {
    c0 -= other.c0;
    c1 -= other.c1;
    c2 -= other.c2;
    return *this;
  }

  /**
   * Multiplication of this matrix by vector.
   *
   * \param  vector  The vector this one is multiplied by
   * \return         A new vector containing the result
   *                 of the calculation.
   */
  Vector3<V> operator*(const Vector3<V>& vector) const {
    /*
    return c0 * vector.x + c1 * vector.y + c2 * vector.z;
    */
    return Vector3<V>(c0.x * vector.x + c1.x * vector.y + c2.x * vector.z,
                      c0.y * vector.x + c1.y * vector.y + c2.y * vector.z,
                      c0.z * vector.x + c1.z * vector.y + c2.z * vector.z);
  }

  /**
   * Multiplication of this matrix by another matrix.
   *
   * \param  other  The other matrix this one is multiplied by
   * \return        A new matrix containing the result
   *                of the calculation.
   */
  Matrix3x3<V> operator*(const Matrix3x3<V>& other) const {
    // this method is up to 2 times faster than "return Matrix3x3<V>((*this) * other.c0, (*this) * other.c1, (*this) *
    // other.c2);"
    Matrix3x3<V> result;
    result.c0.x = c0.x * other.c0.x + c1.x * other.c0.y + c2.x * other.c0.z;
    result.c0.y = c0.y * other.c0.x + c1.y * other.c0.y + c2.y * other.c0.z;
    result.c0.z = c0.z * other.c0.x + c1.z * other.c0.y + c2.z * other.c0.z;
    result.c1.x = c0.x * other.c1.x + c1.x * other.c1.y + c2.x * other.c1.z;
    result.c1.y = c0.y * other.c1.x + c1.y * other.c1.y + c2.y * other.c1.z;
    result.c1.z = c0.z * other.c1.x + c1.z * other.c1.y + c2.z * other.c1.z;
    result.c2.x = c0.x * other.c2.x + c1.x * other.c2.y + c2.x * other.c2.z;
    result.c2.y = c0.y * other.c2.x + c1.y * other.c2.y + c2.y * other.c2.z;
    result.c2.z = c0.z * other.c2.x + c1.z * other.c2.y + c2.z * other.c2.z;
    return result;
  }

  /**
   * Multiplication of this matrix by another matrix.
   *
   * \param  other  The other matrix this one is multiplied by
   * \return        A reference this object after the calculation.
   */
  Matrix3x3<V>& operator*=(const Matrix3x3<V>& other) {
    // this method is somehow faster than "return *this = *this * other;"
    Matrix3x3<V> result;
    result.c0.x = c0.x * other.c0.x + c1.x * other.c0.y + c2.x * other.c0.z;
    result.c0.y = c0.y * other.c0.x + c1.y * other.c0.y + c2.y * other.c0.z;
    result.c0.z = c0.z * other.c0.x + c1.z * other.c0.y + c2.z * other.c0.z;
    result.c1.x = c0.x * other.c1.x + c1.x * other.c1.y + c2.x * other.c1.z;
    result.c1.y = c0.y * other.c1.x + c1.y * other.c1.y + c2.y * other.c1.z;
    result.c1.z = c0.z * other.c1.x + c1.z * other.c1.y + c2.z * other.c1.z;
    result.c2.x = c0.x * other.c2.x + c1.x * other.c2.y + c2.x * other.c2.z;
    result.c2.y = c0.y * other.c2.x + c1.y * other.c2.y + c2.y * other.c2.z;
    result.c2.z = c0.z * other.c2.x + c1.z * other.c2.y + c2.z * other.c2.z;
    *this = result;
    return *this;
  }

  /**
   * Multiplication of this matrix by a factor.
   *
   * \param  factor  The factor this matrix is multiplied by
   * \return         A reference to this object after the calculation.
   */
  Matrix3x3<V>& operator*=(const V& factor) {
    c0 *= factor;
    c1 *= factor;
    c2 *= factor;
    return *this;
  }

  /**
   * Division of this matrix by a factor.
   *
   * \param  factor  The factor this matrix is divided by
   * \return         A reference to this object after the calculation.
   */
  Matrix3x3<V>& operator/=(const V& factor) {
    c0 /= factor;
    c1 /= factor;
    c2 /= factor;
    return *this;
  }

  /**
   * Multiplication of this matrix by a factor.
   *
   * \param  factor  The factor this matrix is multiplied by
   * \return         A new object that contains the result of the calculation.
   */
  Matrix3x3<V> operator*(const V& factor) const { return Matrix3x3<V>(c0 * factor, c1 * factor, c2 * factor); }

  /**
   * Division of this matrix by a factor.
   *
   * \param  factor  The factor this matrix is divided by
   * \return         A new object that contains the result of the calculation.
   */
  Matrix3x3<V> operator/(const V& factor) const { return Matrix3x3<V>(*this) /= factor; }

  /**
   * Comparison of another matrix with this one.
   *
   * \param  other  The other matrix that will be compared to this one
   * \return        Whether the two matrices are equal.
   */
  bool operator==(const Matrix3x3<V>& other) const { return c0 == other.c0 && c1 == other.c1 && c2 == other.c2; }

  /**
   * Comparison of another matrix with this one.
   *
   * \param  other  The other matrix that will be compared to this one
   * \return        Whether the two matrixs are unequal.
   */
  bool operator!=(const Matrix3x3<V>& other) const { return c0 != other.c0 || c1 != other.c1 || c2 != other.c2; }

  /**
   * Array-like member access.
   * \param  i index
   * \return reference to column
   */
  Vector3<V>& operator[](int i) { //
    if (i == 0) {
      return c0;
    } else if (i == 1) {
      return c1;
    } else if (i == 2) {
      return c2;
    }
    std::abort();
  }

  /**
   * const array-like member access.
   * \param  i index
   * \return reference to column
   */
  const Vector3<V>& operator[](int i) const {
    if (i == 0) {
      return c0;
    } else if (i == 1) {
      return c1;
    } else if (i == 2) {
      return c2;
    }
    std::abort();
  }

  /**
   * Transpose the matrix
   *
   * \return  A new object containing transposed matrix
   */
  Matrix3x3<V> transpose() const {
    return Matrix3x3<V>(Vector3<V>(c0.x, c1.x, c2.x), Vector3<V>(c0.y, c1.y, c2.y), Vector3<V>(c0.z, c1.z, c2.z));
  }

  /**
   * Calculation of the determinant of this matrix.
   *
   * \return The determinant.
   */
  V det() const {
    return c0.x * (c1.y * c2.z - c1.z * c2.y) + c0.y * (c1.z * c2.x - c1.x * c2.z) + c0.z * (c1.x * c2.y - c1.y * c2.x);
  }

  /**
   * Calculate determinant of 2x2 Submatrix
   * | a b |
   * | c d |
   *
   * \return  determinant.
   */
  static V det2(V a, V b, V c, V d) { return a * d - b * c; }

  /**
   * Calculate the adjoint of this matrix.
   *
   * \return the adjoint matrix.
   */
  Matrix3x3<V> adjoint() const {
    return Matrix3x3<V>(
      Vector3<V>(det2(c1.y, c2.y, c1.z, c2.z), det2(c2.x, c1.x, c2.z, c1.z), det2(c1.x, c2.x, c1.y, c2.y)),
      Vector3<V>(det2(c2.y, c0.y, c2.z, c0.z), det2(c0.x, c2.x, c0.z, c2.z), det2(c2.x, c0.x, c2.y, c0.y)),
      Vector3<V>(det2(c0.y, c1.y, c0.z, c1.z), det2(c1.x, c0.x, c1.z, c0.z), det2(c0.x, c1.x, c0.y, c1.y)));
  }

  /**
   * Calculate the inverse of this matrix.
   *
   * \return The inverse matrix
   */
  Matrix3x3<V> invert() const { return adjoint().transpose() / det(); }
};
