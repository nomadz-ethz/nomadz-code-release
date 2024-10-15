#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "nao_lola_sensor_msgs/msg/fsr.hpp"
#include "nomadz_definitions/limbs.hpp"
#include "nomadz_kinematics/robot_model.hpp"
#include "nomadz_proprioception_msgs/msg/robot_model.hpp"

#include "nomadz_proprioception/types.hpp"

namespace nomadz_proprioception {

  geometry_msgs::msg::TransformStamped packTransformStamped(const Eigen::Affine3f& transform,
                                                            std::string_view frame_id,
                                                            std::string_view child_frame_id,
                                                            builtin_interfaces::msg::Time stamp);

  geometry_msgs::msg::QuaternionStamped packQuaternionStamped(const Eigen::Quaternionf& quaternion,
                                                              std::string_view frame_id,
                                                              builtin_interfaces::msg::Time stamp);

  FsrArray unpackFsr(const nao_lola_sensor_msgs::msg::Fsr& msg);

  inline Timestamp toTimestamp(const builtin_interfaces::msg::Time& stamp) {
    return std::chrono::steady_clock::time_point(std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec));
  }
} // namespace nomadz_proprioception
