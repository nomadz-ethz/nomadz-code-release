/**
 * @file Matrix2x2.h
 *
 * Contains template class Matrix2x2 of type V
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:Kai_Engel@gmx.de">Kai Engel</a>
 * and <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Vector2.h"
#include <limits>

namespace internal {
  template <typename T> class matrix2x2_ros_wrapper : public ROSMessage<void, false> {};
} // namespace internal

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/matrix2x2.hpp"
namespace internal {
  template <> class matrix2x2_ros_wrapper<float> : public ROSMessage<nomadz_msgs::msg::Matrix2x2> {
  public:
    using base_type = nomadz_msgs::msg::Matrix2x2;
  };
} // namespace internal
#endif

/** This class represents a 2x2-matrix */
template <class V = float> class Matrix2x2 : public Streamable, public internal::matrix2x2_ros_wrapper<V> {
private:
  virtual void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(c0);
    STREAM(c1);
    STREAM_REGISTER_FINISH;
  }

public:
  /** The columns of the matrix */
  Vector2<V> c0, c1;

  /** Default constructor. */
#ifdef ENABLE_ROS
  Matrix2x2()
      : c0(internal::matrix2x2_ros_wrapper<V>::base_type::c0, internal::ref_init_tag{}),
        c1(internal::matrix2x2_ros_wrapper<V>::base_type::c1, internal::ref_init_tag{}) {
    c0 = Vector2<V>(1, 0);
    c1 = Vector2<V>(0, 1);
  }
#else
  Matrix2x2() {
    c0 = Vector2<V>(1, 0);
    c1 = Vector2<V>(0, 1);
  }
#endif
  template <typename U> inline Matrix2x2(U& ref, internal::ref_init_tag t) : c0(ref.c0, t), c1(ref.c1, t) {
    c0 = Vector2<V>(1, 0);
    c1 = Vector2<V>(0, 1);
  }
  template <typename U> inline Matrix2x2(U& ref, internal::ref_alias_tag t) : c0(ref.c0, t), c1(ref.c1, t) {}

  /**
   * Anti-lateral thinking constructor.
   */
  Matrix2x2(const V& a11, const V& a12, const V& a21, const V& a22) : Matrix2x2() {
    c0.x = a11;
    c1.x = a12;
    c0.y = a21;
    c1.y = a22;
  }

  //! Constructor
  /*!
  \param c0 the first column of the matrix.
  \param c1 the second column of the matrix.
  */
  Matrix2x2(const Vector2<V>& ic0, const Vector2<V>& ic1) : Matrix2x2() {
    c0 = ic0;
    c1 = ic1;
  }

  //! Assignment operator
  /*!
  \param other The other matrix that is assigned to this one
  \return A reference to this object after the assignment.
  */
  Matrix2x2& operator=(const Matrix2x2& other) {
    c0 = other.c0;
    c1 = other.c1;
    return *this;
  }

  //! Copy constructor
  /*!
  \param other The other matrix that is copied to this one
   */
  Matrix2x2(const Matrix2x2& other) : Matrix2x2() { *this = other; }

  /**
   * Array-like member access.
   * \param  i index
   * \return reference to column
   */
  Vector2<V>& operator[](int i) {
    if (i == 0) {
      return c0;
    } else if (i == 1) {
      return c1;
    }
    std::abort();
  }

  /**
   * const array-like member access.
   * \param  i index
   * \return reference to column
   */
  const Vector2<V>& operator[](int i) const {
    if (i == 0) {
      return c0;
    } else if (i == 1) {
      return c1;
    }
    std::abort();
  }

  //! Multiplication of this matrix by vector.
  /*!
  \param vector The vector this one is multiplied by
  \return A reference to a new vector containing the result
    of the calculation.
  */
  Vector2<V> operator*(const Vector2<V>& vector) const { return (c0 * vector.x + c1 * vector.y); }

  //! Multiplication of this matrix by another matrix.
  /*!
  \param other The other matrix this one is multiplied by
  \return An object containing the result of the calculation.
  */
  Matrix2x2 operator*(const Matrix2x2& other) const {
    Matrix2x2 returnMatrix;
    returnMatrix.c0.x = c0.x * other.c0.x + c1.x * other.c0.y;
    returnMatrix.c0.y = c0.y * other.c0.x + c1.y * other.c0.y;
    returnMatrix.c1.x = c0.x * other.c1.x + c1.x * other.c1.y;
    returnMatrix.c1.y = c0.y * other.c1.x + c1.y * other.c1.y;
    return returnMatrix;
  }

  //! Multiplication of this matrix by another matrix.
  /*!
  \param other The other matrix this one is multiplied by
  \return A reference this object after the calculation.
  */
  Matrix2x2 operator*=(const Matrix2x2& other) { return *this = *this * other; }

  //! Multiplication of this matrix by a factor.
  /*!
  \param factor The factor this matrix is multiplied by
  \return A reference to this object after the calculation.
  */
  Matrix2x2& operator*=(const V& factor) {
    c0 *= factor;
    c1 *= factor;
    return *this;
  }

  //! Division of this matrix by a factor.
  /*!
  \param factor The factor this matrix is divided by
  \return A reference to this object after the calculation.
   */
  Matrix2x2& operator/=(const V& factor) {
    c0 /= factor;
    c1 /= factor;
    return *this;
  }

  //! Multiplication of this matrix by a factor.
  /*!
  \param factor The factor this matrix is multiplied by
  \return A new object that contains the result of the calculation.
  */
  Matrix2x2 operator*(const V& factor) const { return Matrix2x2(*this) *= factor; }

  //! Division of this matrix by a factor.
  /*!
  \param factor The factor this matrix is divided by
  \return A new object that contains the result of the calculation.
  */
  Matrix2x2 operator/(const V& factor) const { return Matrix2x2(*this) /= factor; }

  //! Computes the sum of two matrices
  /*!
  \param other Another matrix
  \return The sum
  */
  Matrix2x2 operator+(const Matrix2x2& other) const {
    return Matrix2x2(Vector2<V>(c0.x + other.c0.x, c0.y + other.c0.y), Vector2<V>(c1.x + other.c1.x, c1.y + other.c1.y));
  }

  //! Computes the difference of two matrices
  /*!
  \param other Another matrix
  \return The difference
  */
  Matrix2x2 operator-(const Matrix2x2& other) const {
    return Matrix2x2(Vector2<V>(c0.x - other.c0.x, c0.y - other.c0.y), Vector2<V>(c1.x - other.c1.x, c1.y - other.c1.y));
  }

  /**
   * Adds another matrix.
   *
   * \param  other  The other matrix that is added to this one
   * \return        A reference to this object after the calculation.
   */
  Matrix2x2& operator+=(const Matrix2x2& other) {
    c0 += other.c0;
    c1 += other.c1;
    return *this;
  }

  /**
   * Subtracts another matrix.
   *
   * \param  other  The other matrix that is subtracted from this one
   * \return        A reference to this object after the calculation.
   */
  Matrix2x2& operator-=(const Matrix2x2& other) {
    c0 -= other.c0;
    c1 -= other.c1;
    return *this;
  }

  //! Computes an inverted matrix.
  /*!
  \return An inverted matrix.
  */
  Matrix2x2 invert() const {
    V factor(det());
    if (std::abs(factor) < std::numeric_limits<V>::min())
      factor = std::numeric_limits<V>::min();
    else
      factor = 1.f / factor;
    return Matrix2x2(Vector2<V>(factor * c1.y, -factor * c0.y), Vector2<V>(-factor * c1.x, factor * c0.x));
  }

  //! Comparison of another matrix with this one.
  /*!
  \param other The other matrix that will be compared to this one
  \return Whether the two matrices are equal.
  */
  bool operator==(const Matrix2x2& other) const { return (c0 == other.c0 && c1 == other.c1); }

  //! Comparison of another matrix with this one.
  /*!
  \param other The other matrix that will be compared to this one
  \return Whether the two matrixs are unequal.
  */
  bool operator!=(const Matrix2x2& other) const { return !(*this == other); }

  /*! Transpose the matrix
  \return A new object containing transposed matrix
  */
  Matrix2x2 transpose() const { return Matrix2x2(Vector2<V>(c0.x, c1.x), Vector2<V>(c0.y, c1.y)); }

  Matrix2x2 absDiagonal() const { return Matrix2x2(Vector2<V>(std::abs(c0.x), c0.y), Vector2<V>(c1.x, std::abs(c1.y))); }

  // Rotate a matrix M by rotation matrix R: New M = RMR'
  Matrix2x2 rotate(float angle) const {
    Matrix2x2 rotationMatrix;
    rotationMatrix.c0.x = cosf(angle);
    rotationMatrix.c1.x = -sinf(angle);
    rotationMatrix.c0.y = sinf(angle);
    rotationMatrix.c1.y = cosf(angle);

    Matrix2x2 M;
    M.c0.x = c0.x;
    M.c1.x = c1.x;
    M.c0.y = c0.y;
    M.c1.y = c1.y;

    Matrix2x2 RM = rotationMatrix * M;
    Matrix2x2 RT = rotationMatrix.transpose();
    return RM * RT;
  }

  //! Calculation of the determinant of this matrix.
  /*!
  \return The determinant.
  */
  V det() const { return c0.x * c1.y - c1.x * c0.y; }

  /** Sums up the elements on the main diagonal
   * @return The sum
   */
  V trace() const { return c0.x + c1.y; }
};
