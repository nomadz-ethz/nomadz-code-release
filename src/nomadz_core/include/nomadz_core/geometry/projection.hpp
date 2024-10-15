#pragma once

#include <Eigen/Geometry>
#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_core/geometry/rotation.hpp"
#include "nomadz_core/geometry/line.hpp"

namespace nomadz_core {
  inline Eigen::Affine2f projectTo2D(const Eigen::Affine3f& pose) {
    Eigen::Affine2f pose2d = Eigen::Affine2f::Identity();
    pose2d.translation() = pose.translation().head<2>();
    pose2d.linear() = pose.linear().topLeftCorner<2, 2>();
    return pose2d;
  }

  inline Pose2D projectTo2D(const Pose& pose) {
    Pose2D pose2d{};
    pose2d.x = pose.translation.x();
    pose2d.y = pose.translation.y();
    pose2d.theta = angle::normalize(getRotationRelativeZAngle(pose.orientation.toRotationMatrix()));
    return pose2d;
  }

  inline Eigen::Affine3f expandTo3D(const Eigen::Affine2f& affine2, float z = 0.F) {
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    pose.translation() << affine2.translation().x(), affine2.translation().y(), z;
    pose.linear().topLeftCorner<2, 2>() = affine2.linear();
    pose.linear().row(2) << 0, 0, 1;
    return pose;
  }

  inline Pose expandTo3D(const Pose2D& pose2d, float z = 0.F) {
    Pose pose{};
    pose.translation << pose2d.x, pose2d.y, z;
    pose.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(pose2d.theta, Eigen::Vector3f::UnitZ()));
    return pose;
  }

  inline Eigen::Vector2f getOrthogonalProjectionOfPointOnLine(const Eigen::Vector2f& base,
                                                              const Eigen::Vector2f& dir,
                                                              const Eigen::Vector2f& point) {
    const float l = (point.x() - base.x()) * dir.x() + (point.y() - base.y()) * dir.y();
    return base + (dir * l);
  }

  /**
   * Computes the projection of a point on a line
   * @param line The line to project on
   * @param point The point that is projected
   * @return A point on the line
   */
  inline Eigen::Vector2f getOrthogonalProjectionOfPointOnLine(const Line& line, const Eigen::Vector2f& point) {
    return getOrthogonalProjectionOfPointOnLine(line.base, line.direction.normalized(), point);
  }

  /**
   * Computes the projection of a point on an edge
   * @param base The base point of the line
   * @param dir The direction vector of the line (must NOT be normalized)
   * @param point The point that is projected
   * @return A point on the edge
   */
  inline Eigen::Vector2f getOrthogonalProjectionOfPointOnEdge(const Eigen::Vector2f& base,
                                                              const Eigen::Vector2f& dir,
                                                              const Eigen::Vector2f& point) {
    Eigen::Vector2f projection = getOrthogonalProjectionOfPointOnLine(base, dir.normalized(), point);

    const float d = (projection - base).dot(dir) / dir.dot(dir);

    if (d < 0) {
      return base;
    }
    if (d > 1.0F) {
      return base + dir;
    }
    return projection;
  }

  /**
   * Computes the projection of a point on an edge
   * @param line The line to project on
   * @param point The point that is projected
   * @return A point on the edge
   */
  inline Eigen::Vector2f getOrthogonalProjectionOfPointOnEdge(const Line& line, const Eigen::Vector2f& point) {
    return getOrthogonalProjectionOfPointOnEdge(line.base, line.direction, point);
  }

} // namespace nomadz_core
