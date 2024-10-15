#pragma once

#include <Eigen/Geometry>

#include "nomadz_core/math/approx.hpp"

namespace nomadz_core {
  /**
   * @brief Represents a twist in 3D space.
   *
   * This struct contains two members: linear and angular, which represent the linear and angular components of the twist,
   * respectively.
   */
  struct Twist {
    // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
    Eigen::Vector3f linear;  /**< The linear component of the twist. */
    Eigen::Vector3f angular; /**< The angular component of the twist. */
    // NOLINTEND(misc-non-private-member-variables-in-classes)
  };

  /**
   * @brief Represents a 2D twist. The object is also used to represent difference between poses.
   */
  struct Twist2D {
    // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
    float x;     /**< The linear velocity along the x-axis (m/s) or dx (m) for pose difference. */
    float y;     /**< The linear velocity along the y-axis (m/s) or dy (m) for pose difference. */
    float theta; /**< The angular velocity around the z-axis (rad/s) or dtheta (rad) for pose difference. */
    // NOLINTEND(misc-non-private-member-variables-in-classes)

    Twist2D operator+=(const Twist2D& rhs) {
      x += rhs.x;
      y += rhs.y;
      theta += rhs.theta;
      return *this;
    }

    Twist2D operator-=(const Twist2D& rhs) {
      x -= rhs.x;
      y -= rhs.y;
      theta -= rhs.theta;
      return *this;
    }

    Twist2D operator*=(float scalar) {
      x *= scalar;
      y *= scalar;
      theta *= scalar;
      return *this;
    }

    Twist2D operator/=(float scalar) {
      x /= scalar;
      y /= scalar;
      theta /= scalar;
      return *this;
    }
  };

  /**
   * @brief Overloaded equality operator for comparing two Twist2D objects.
   *
   * @param lhs The left-hand side Twist2D object.
   * @param rhs The right-hand side Twist2D object.
   * @return True if the x, y, and theta values of the two Twist2D objects are equal, false otherwise.
   */
  inline bool operator==(const Twist2D& lhs, const Twist2D& rhs) {
    return approx::isEqual(lhs.x, rhs.x) && approx::isEqual(lhs.y, rhs.y) && approx::isEqual(lhs.theta, rhs.theta);
  }

  inline bool operator!=(const Twist2D& lhs, const Twist2D& rhs) {
    return !(lhs == rhs);
  }

  /**
   * @brief Adds two Twist2D objects.
   * @param lhs The left-hand side Twist2D.
   * @param rhs The right-hand side Twist2D.
   * @return The resulting Twist2D after addition.
   */
  inline Twist2D operator+(const Twist2D& lhs, const Twist2D& rhs) {
    return Twist2D{lhs.x + rhs.x, lhs.y + rhs.y, lhs.theta + rhs.theta};
  }

  /**
   * @brief Subtracts one Twist2D object from another.
   * @param lhs The left-hand side Twist2D.
   * @param rhs The right-hand side Twist2D.
   * @return The resulting Twist2D after subtraction.
   */
  inline Twist2D operator-(const Twist2D& lhs, const Twist2D& rhs) {
    return Twist2D{lhs.x - rhs.x, lhs.y - rhs.y, lhs.theta - rhs.theta};
  }

  /**
   * @brief Multiplies a Twist2D object by a scalar value.
   * @param lhs The left-hand side Twist2D.
   * @param scalar The scalar value to multiply by.
   * @return The resulting Twist2D after multiplication.
   */
  inline Twist2D operator*(const Twist2D& lhs, float scalar) {
    return Twist2D{lhs.x * scalar, lhs.y * scalar, lhs.theta * scalar};
  }

  /**
   * @brief Divides a Twist2D object by a scalar value.
   * @param lhs The left-hand side Twist2D.
   * @param scalar The scalar value to divide by.
   * @return The resulting Twist2D after division.
   */
  inline Twist2D operator/(const Twist2D& lhs, float scalar) {
    return Twist2D{lhs.x / scalar, lhs.y / scalar, lhs.theta / scalar};
  }

  inline Eigen::Vector3f toVector3(const Twist2D& twist) {
    return Eigen::Vector3f{twist.x, twist.y, twist.theta};
  }

  inline Twist2D toTwist2D(const Eigen::Vector3f& vec) {
    return Twist2D{vec.x(), vec.y(), vec.z()};
  }
} // namespace nomadz_core
