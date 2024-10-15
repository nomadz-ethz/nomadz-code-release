#pragma once

#include <cmath>
#include <Eigen/Geometry>

#include "nomadz_core/geometry/angle.hpp"
#include "nomadz_core/geometry/twist.hpp"
#include "nomadz_core/math/approx.hpp"

namespace nomadz_core {
  /**
   * @brief Represents a 3D pose in Cartesian coordinates.
   */
  struct Pose {
    // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
    Eigen::Vector3f translation;    /**< The translation vector of the pose. */
    Eigen::Quaternionf orientation; /**< The orientation quaternion of the pose. */
    // NOLINTEND(misc-non-private-member-variables-in-classes)
  };

  /**
   * @brief Represents a 2D pose in Cartesian coordinates.
   * @note The Twist2D here is also treated as both differential pose.
   */
  struct Pose2D {
    // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
    float x;     /**< The x-coordinate of the pose. */
    float y;     /**< The y-coordinate of the pose. */
    float theta; /**< The orientation angle [-pi, pi] of the pose. */
    // NOLINTEND(misc-non-private-member-variables-in-classes)

    Pose2D operator+=(const Twist2D& rhs) {
      x += rhs.x;
      y += rhs.y;
      theta = angle::normalize(theta + rhs.theta);
      return *this;
    }

    Pose2D operator-=(const Twist2D& rhs) {
      x -= rhs.x;
      y -= rhs.y;
      theta = angle::normalize(theta - rhs.theta);
      return *this;
    }
  };

  /**
   * @brief Overloaded equality operator for comparing two Pose2D objects.
   *
   * @param lhs The left-hand side Pose2D object.
   * @param rhs The right-hand side Pose2D object.
   * @return True if the x, y, and theta values of the two Pose2D objects are equal, false otherwise.
   */
  inline bool operator==(const Pose2D& lhs, const Pose2D& rhs) {
    return approx::isEqual(lhs.x, rhs.x) && approx::isEqual(lhs.y, rhs.y) && approx::isEqual(lhs.theta, rhs.theta);
  }

  /**
   * @brief Compute the differential pose between two poses.
   * @param lhs The first pose.
   * @param rhs The second pose.
   * @return The differential pose as a Twist2D object.
   */
  inline Twist2D operator-(const Pose2D& lhs, const Pose2D& rhs) {
    return Twist2D{lhs.x - rhs.x, lhs.y - rhs.y, angle::normalize(lhs.theta - rhs.theta)};
  }

  /**
   * @brief Subtract a Twist2D from a Pose2D.
   * @param lhs The Pose2D.
   * @param rhs The Twist2D.
   * @return The updated pose after subtraction.
   */
  inline Pose2D operator-(const Pose2D& lhs, const Twist2D& rhs) {
    return Pose2D{lhs.x - rhs.x, lhs.y - rhs.y, angle::normalize(lhs.theta - rhs.theta)};
  }

  /**
   * @brief Add a Twist2D to a Pose2D.
   * @param lhs The Pose2D.
   * @param rhs The Twist2D.
   * @return The updated pose after addition.
   */
  inline Pose2D operator+(const Pose2D& lhs, const Twist2D& rhs) {
    return Pose2D{lhs.x + rhs.x, lhs.y + rhs.y, angle::normalize(lhs.theta + rhs.theta)};
  }

  /**
   * @brief Convert a Pose2D to an Affine2f transformation matrix.
   * @param pose The Pose2D to be converted.
   * @return The Affine2f transformation matrix.
   */
  inline Eigen::Affine2f toAffine2(const Pose2D& pose) {
    Eigen::Affine2f affine;
    affine.translation() = Eigen::Vector2f{pose.x, pose.y};
    affine.linear() = Eigen::Rotation2Df{pose.theta}.toRotationMatrix();
    return affine;
  }

  /**
   * @brief Convert an Affine2f transformation matrix to a Pose2D.
   * @param affine The Affine2f transformation matrix.
   * @return The converted Pose2D.
   */
  inline Pose2D toPose2D(const Eigen::Affine2f& affine) {
    const float theta = std::atan2(affine.linear()(1, 0), affine.linear()(0, 0));
    return Pose2D{affine.translation().x(), affine.translation().y(), theta};
  }

  /**
   * @brief Convert a Pose2D to a 3D Eigen vector.
   * @param pose The Pose2D to be converted.
   * @return The converted 3D Eigen vector.
   */
  inline Eigen::Vector3f toVector3(const Pose2D& pose) {
    return Eigen::Vector3f{pose.x, pose.y, pose.theta};
  }

  /**
   * @brief Convert a 3D Eigen vector to a Pose2D.
   * @param vec The 3D vector to be converted.
   * @return The converted Pose2D.
   */
  inline Pose2D toPose2D(const Eigen::Vector3f& vec) {
    return Pose2D{vec.x(), vec.y(), vec.z()};
  }

  /**
   * @brief Convert a std::array vector to a Pose2D.
   * @param vec The array to be converted.
   * @return The converted Pose2D.
   */
  inline Pose2D toPose2D(const std::array<float, 3>& vec) {
    return Pose2D{vec.at(0), vec.at(1), vec.at(2)};
  }

  /**
   * @brief Compute the 2D Euclidean distance between two Pose2D objects.
   * @param lhs The first Pose2D object.
   * @param rhs The second Pose2D object.
   * @return The distance between the two Pose2D objects.
   */
  inline float distance(const Pose2D& lhs, const Pose2D& rhs) {
    return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
  }

} // namespace nomadz_core
