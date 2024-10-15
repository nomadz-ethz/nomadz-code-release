#include "nomadz_proprioception/ros_conversion.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include "nomadz_definitions/limbs.hpp"

namespace side = nomadz_definitions::side;

namespace nomadz_proprioception {

  geometry_msgs::msg::TransformStamped packTransformStamped(const Eigen::Affine3f& transform,
                                                            std::string_view frame_id,
                                                            std::string_view child_frame_id,
                                                            builtin_interfaces::msg::Time stamp) {
    geometry_msgs::msg::TransformStamped msg = tf2::eigenToTransform(transform.cast<double>());
    msg.header.frame_id = frame_id;
    msg.child_frame_id = child_frame_id;
    msg.header.stamp = stamp;
    return msg;
  }

  geometry_msgs::msg::QuaternionStamped packQuaternionStamped(const Eigen::Quaternionf& quaternion,
                                                              std::string_view frame_id,
                                                              builtin_interfaces::msg::Time stamp) {
    geometry_msgs::msg::QuaternionStamped msg;
    msg.header.frame_id = frame_id;
    msg.quaternion = tf2::toMsg(quaternion.cast<double>());
    msg.header.stamp = stamp;
    return msg;
  }

  FsrArray unpackFsr(const nao_lola_sensor_msgs::msg::Fsr& msg) {
    FsrArray fsr;

    fsr[side::LEFT][Fsr::FRONT_LEFT] = msg.l_foot_front_left;
    fsr[side::LEFT][Fsr::FRONT_RIGHT] = msg.l_foot_front_right;
    fsr[side::LEFT][Fsr::BACK_LEFT] = msg.l_foot_back_left;
    fsr[side::LEFT][Fsr::BACK_RIGHT] = msg.l_foot_back_right;

    fsr[side::RIGHT][Fsr::FRONT_LEFT] = msg.r_foot_front_left;
    fsr[side::RIGHT][Fsr::FRONT_RIGHT] = msg.r_foot_front_right;
    fsr[side::RIGHT][Fsr::BACK_LEFT] = msg.r_foot_back_left;
    fsr[side::RIGHT][Fsr::BACK_RIGHT] = msg.r_foot_back_right;

    return fsr;
  }
} // namespace nomadz_proprioception
