#pragma once

#include <string_view>
#include <Eigen/Core>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_core/geometry/twist.hpp"

namespace nomadz_core {
  Eigen::Vector3f unpackVector3(geometry_msgs::msg::Vector3 vec);
  geometry_msgs::msg::Vector3 packVector3(Eigen::Vector3f vec);

  Pose2D unpackPose2D(geometry_msgs::msg::Pose2D pose);
  geometry_msgs::msg::Pose2D packPose2D(Pose2D pose);

  Twist2D unpackTwist2D(geometry_msgs::msg::Twist twist);
  geometry_msgs::msg::Twist packTwist2D(Twist2D twist);

  geometry_msgs::msg::Point packPoint(const Eigen::Vector2d& vec);
  geometry_msgs::msg::Point packPoint(const Eigen::Vector3d& vec);

  geometry_msgs::msg::Transform packTransform(const Eigen::Affine3f& transform);

  geometry_msgs::msg::TransformStamped packTransformStamped(builtin_interfaces::msg::Time stamp,
                                                            std::string_view frame_id,
                                                            std::string_view child_frame_id,
                                                            const Eigen::Affine3f& transform);
} // namespace nomadz_core
