/**
 * @file Point.h
 *
 * This file contains the Point class.
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once

#include "math.h"
#include <cmath>
#include <string>
#ifndef WALKING_SIMULATOR
#include "Core/Math/Vector.h"
#include <Eigen/Eigen>
#include "Core/Math/BHMath.h"
#include "Core/Math/Pose2D.h"
#include "Core/Streams/AutoStreamable.h"
#else
#include "math/Vector3.h"
#endif

enum Dimension { X, Y, Z, RX, RY, RZ };

/**
 * @class Point
 * Class representing a 3D Point.
 */
class Point {
public:
  /**
   * Constructor. Creates a point at coordinates (x, y, 0).
   * @param x x coordinate.
   * @param y y coordinate.
   */
  Point(float x, float y) {
    this->x = x;
    this->y = y;
    r = z = 0;
    rx = ry = 0;
  }

#ifndef WALKING_SIMULATOR

  /**
   * Constructor. Creates a point at coordinates (v[0], v[1], v[3]).
   * @param v The vector to create from.
   */
  Point(Eigen::Vector3f v) {
    x = v[0];
    y = v[1];
    z = v[2];
    r = rx = ry = 0;
  }

  Point(Pose2D& other) {
    x = other.translation.x;
    y = other.translation.y;
    z = rx = ry = 0;
    r = other.rotation;
  }

  Point(const Pose2D& other) {
    x = other.translation.x;
    y = other.translation.y;
    z = rx = ry = 0;
    r = other.rotation;
  }

  operator Pose2D() const { return Pose2D(r, x, y); }

#endif

  /**
   * Constructor. Creates a point at coordinates (x, y, z).
   * @param x x coordinate.
   * @param y y coordinate.
   * @param z z coordinate.
   */
  Point(float x, float y, float z) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->r = 0;
    rx = ry = 0;
  }

  /**
   * Constructor. Creates a point at coordinates (x, y, z) with rotation r around the local z axis.
   * @param x x coordinate.
   * @param y y coordinate.
   * @param z z coordinate.
   * @param r Rotation around local z axis.
   */
  Point(float x, float y, float z, float r) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->r = r;
    rx = ry = 0;
  }

  Point(float x, float y, float z, float r, float rx, float ry) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->r = r;
    this->rx = rx;
    this->ry = ry;
  }

  bool operator==(const Point& p) const { return x == p.x && y == p.y && z == p.z && r == p.r && rx == p.rx && ry == p.ry; }

  /** Constructor. Sets all to 0. */
  Point() { x = y = z = r = rx = ry = 0; }

  /** Set new value using an IIR filter */

  void newValue(Point v, float alpha) { *this = *this * (1.f - alpha) + v * alpha; }

  std::string toString() const {
    using std::to_string;
    return to_string(x) + ", " + to_string(y) + ", " + to_string(z) + ", " + to_string(r) + ", " + to_string(rx) + ", " +
           to_string(ry);
  }

  /**
   * Copy data from another point.
   * @param p The source point
   * @return Copy of the instance
   */
  Point& operator=(const Point& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    r = p.r;
    rx = p.rx;
    ry = p.ry;
    return *this;
  }

  /**
   * Multiply operator. Multiplies every component with the corresponding component of p.
   * @param p The point to multiply with.
   * @return The result.
   */
  Point operator*(const Point& p) {
    Point ret;
    ret.x = x * p.x;
    ret.y = y * p.y;
    ret.z = z * p.z;
    ret.rx = rx * p.rx;
    ret.ry = ry * p.ry;
    ret.r = r * p.r;
    return ret;
  }

  /**
   * Multiply operator. Multiplies every component with the corresponding
   * component of p and stores the result in this instance.
   * @param p The point to multiply with.
   */
  void operator*=(const Point& p) { *this = *this * p; }

  /**
   * Multiply with scalar. Multiplies every component with the scalar.
   * @param f The scalar to multiply with.
   * @return Copy of the instance.
   */
  Point operator*(const float f) {
    Point ret;
    ret.x = x * f;
    ret.y = y * f;
    ret.z = z * f;
    ret.rx = rx * f;
    ret.ry = ry * f;
    ret.r = r * f;
    return ret;
  }

  /**
   * Division by f. Divides every component by the scalar.
   * @param f The scalar.
   * @return Copy of the instance.
   */
  Point operator/(const float f) { return *this * (1 / f); }

  /**
   * Multiply with scalar. Multiplies every component with the scalar and stores the result in this instance.
   * @param f The scalar to multiply with.
   * @return Copy of the instance.
   */
  Point operator*=(const float f) {
    *this = *this * f;
    return *this * f;
  }

  /**
   * Adds another point.
   * @param p The other point.
   * @return Copy of the instance.
   */
  Point operator+(const Point& p) {
    Point ret;
    ret.x = x + p.x;
    ret.y = y + p.y;
    ret.z = z + p.z;
    ret.rx = rx + p.rx;
    ret.ry = ry + p.ry;
    ret.r = r + p.r;
    return ret;
  }

  /**
   * Substracts another point.
   * @param p The other point.
   * @return Copy of the instance.
   */
  Point operator-(const Point& p) const {
    Point ret;
    ret.x = x - p.x;
    ret.y = y - p.y;
    ret.z = z - p.z;
    ret.rx = rx - p.rx;
    ret.ry = ry - p.ry;
    ret.r = r - p.r;
    return ret;
  }

  /**
   * Sets every component to the scalar.
   * @param p The scaler.
   */
  void operator=(float p) { r = rx = ry = x = y = z = p; }

  /**
   * To set data from a bhuman Vector2<>.
   * @param v The vector.
   */
  void operator=(const Vector2f& v) {
    x = v.x;
    y = v.y;
  }

  /**
   * Checks for equality to another point.
   * @param other The other point.
   * @return True if equal, false otherwise.
   */
  bool operator!=(Point other) {
    return (x != other.x || y != other.y || z != other.z || r != other.r || rx != other.rx || ry != other.ry);
  }

  /**
   * Adds another point and stores the result in this instance.
   * @param p The other point.
   * @return Copy of the instance.
   */
  Point& operator+=(const Point& p) {
    x += p.x;
    y += p.y;
    z += p.z;
    r += p.r;
    rx += p.rx;
    ry += p.ry;
    return *this;
  }

  /**
   * Substracts another point and stores the result in this instance.
   * @param p The other point.
   * @return Copy of the instance.
   */
  Point& operator-=(const Point& p) {
    x -= p.x;
    y -= p.y;
    z -= p.z;
    r -= p.r;
    rx -= p.rx;
    ry -= p.ry;
    return *this;
  }

  /**
   * Calculates the length of the Vector from 0 to p.
   * @param p The other point.
   * @return The distance.
   */
  inline float euklidDistance2D() const {
    Point p;
    return euklidDistance2D(p);
  }

  /**
   * Calculates the euklidian distance to another point within the x-y plane.
   * @param p The other point.
   * @return The distance.
   */
  inline float euklidDistance2D(Point p) const { return std::sqrt(sqr(std::abs(x - p.x)) + sqr(std::abs(y - p.y))); }

  /**
   * Calculates the euklidian distance to another point in 3D space.
   * @param p The other point.
   * @return The distance.
   */
  inline float euklidDistance3D(Point p) {
    return std::sqrt(sqr(std::abs(x - p.x)) + sqr(std::abs(y - p.y)) + sqr(std::abs(z - p.z)));
  }

  /**
   * Rotates the point around the z axis. The z component is therefore constant.
   * @param r Radians.
   * @return Copy of the instance.
   */
  Point rotate2D(float r) {
    float x, y;

    x = std::cos(r) * this->x - std::sin(r) * this->y;
    y = std::sin(r) * this->x + std::cos(r) * this->y;

    this->x = x;
    this->y = y;
    return *this;
  }

  /**
   * Rotates the point around the x axis. The x component is therefore constant.
   * @param r Radians.
   */
  void rotateAroundX(float r) {
    float y, z;
    y = std::cos(r) * this->y - std::sin(r) * this->z;
    z = std::sin(r) * this->y + std::cos(r) * this->z;

    this->y = y;
    this->z = z;
  }

  /**
   * Rotates the point around the y axis. The y component is therefore constant.
   * @param r Radians.
   */
  void rotateAroundY(float r) {
    float x, z;

    x = std::cos(r) * this->x + std::sin(r) * this->z;
    z = -std::sin(r) * this->x + std::cos(r) * this->z;

    this->x = x;
    this->z = z;
  }

  /**
   * Rotates the point around the given unit vector with its root at a given point.
   * @param p The root point to rotate around.
   * @param d The axis (given as unit vector) to rotate around.
   * @param r radians.
   * @return Copy of the instance.
   */
  Point rotate(Point p, Point d, float r) {
    const float cr = std::cos(r);
    const float sr = std::sin(r);

    *this -= p;

    Point t = *this;

    t.x = (cr + d.x * d.x * (1 - cr)) * x + (d.x * d.y * (1 - cr) - d.z * sr) * y + (d.x * d.z * (1 - cr) + d.y * sr) * z;

    t.y = (d.y * d.x * (1 - cr) + d.z * sr) * x + (cr + d.y * d.y * (1 - cr)) * y + (d.y * d.z * (1 - cr) - d.x * sr) * z;

    t.z = (d.z * d.x * (1 - cr) - d.y * sr) * x + (d.z * d.y * (1 - cr) + d.x * sr) * y + (cr + d.z * d.z * (1 - cr)) * z;

    *this = t;

    *this += p;

    return *this;
  }

  /**
   * Scalar product with another point.
   * @param p The other point.
   * @return The result.
   */
  float scalarProduct(Point p) { return x * p.x + y * p.y + z * p.z; }

  /**
   * Calculates the angle between the vectors 0-->this and 0-->vec both
   * projected onto the x-y plane.
   * @param vec The other point.
   * @return Angle in radians.
   */
  float angleTo2D(Point vec) { return std::atan2(vec.y, vec.x) - std::atan2(y, x); }

  /**
   * Calculates the angle between the vectors 0-->this and 0-->vec both
   * in 3D space.
   * @param vec The other point.
   * @return Angle in radians.
   */
  float angleTo3D(Point vec) {
    Point nullPoint;
    return ((*this).scalarProduct(vec)) / ((*this).euklidDistance3D(nullPoint) * vec.euklidDistance3D(nullPoint));
  }

  /**
   * Returns the normalized of the vector 0-->this.
   * @return The length.
   */
  Point norm() const {
    float l = getPositionVecLen();
    return Point(x / l, y / l, z / l);
  }

  /**
   * Calculates the length of the vector 0-->this.
   * @return The length.
   */
  float getPositionVecLen() const { return std::sqrt(x * x + y * y + z * z); }

  union {
    struct {
      float x, /**< x coordinate */
        y,     /**< y coordinate */
        z,     /**< z coordinate */
        rx,    /**< Rotation around x. */
        ry,    /**< Rotation around y. */
        r;     /**< Rotation around z. */
    };
    float v[6];
  };
};

#ifndef WALKING_SIMULATOR

struct StreamPoint : public Point, public Streamable {
  void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(x)
    STREAM(y)
    STREAM(z)
    STREAM(rx)
    STREAM(ry)
    STREAM(r)
    STREAM_REGISTER_FINISH;
  };

public:
  StreamPoint() : Point(), Streamable() {}
  /*
    operator Point()
    {
      return Point(x,y,z,r,rx,ry);
    }
  */
  StreamPoint(const Point& p) : Point(p) {}
};
#endif
