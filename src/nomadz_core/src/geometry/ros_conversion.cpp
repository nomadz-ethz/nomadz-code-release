#include "nomadz_core/geometry/ros_conversion.hpp"

namespace nomadz_core {
  Eigen::Vector3f unpackVector3(geometry_msgs::msg::Vector3 vec) {
    return Eigen::Vector3f{static_cast<float>(vec.x), static_cast<float>(vec.y), static_cast<float>(vec.z)};
  }

  geometry_msgs::msg::Vector3 packVector3(Eigen::Vector3f vec) {
    geometry_msgs::msg::Vector3 vec_msg = geometry_msgs::msg::Vector3();
    vec_msg.x = vec.x();
    vec_msg.y = vec.y();
    vec_msg.z = vec.z();
    return vec_msg;
  }

  geometry_msgs::msg::Vector3 packVector3(const Eigen::Vector2d& vec) {
    geometry_msgs::msg::Vector3 vec_msg = geometry_msgs::msg::Vector3();
    vec_msg.x = vec.x();
    vec_msg.y = vec.y();
    vec_msg.z = 0;
    return vec_msg;
  }

  Pose2D unpackPose2D(geometry_msgs::msg::Pose2D pose) {
    return Pose2D{static_cast<float>(pose.x), static_cast<float>(pose.y), static_cast<float>(pose.theta)};
  }

  geometry_msgs::msg::Pose2D packPose2D(Pose2D pose) {
    geometry_msgs::msg::Pose2D pose_msg = geometry_msgs::msg::Pose2D();
    pose_msg.x = pose.x;
    pose_msg.y = pose.y;
    pose_msg.theta = pose.theta;
    return pose_msg;
  }

  Twist2D unpackTwist2D(geometry_msgs::msg::Twist twist) {
    return Twist2D{
      static_cast<float>(twist.linear.x), static_cast<float>(twist.linear.y), static_cast<float>(twist.angular.z)};
  }

  geometry_msgs::msg::Twist packTwist2D(Twist2D twist) {
    geometry_msgs::msg::Twist twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = twist.x;
    twist_msg.linear.y = twist.y;
    twist_msg.angular.z = twist.theta;
    return twist_msg;
  }

  geometry_msgs::msg::Point packPoint(const Eigen::Vector2d& vec) {
    geometry_msgs::msg::Point point_msg = geometry_msgs::msg::Point();
    point_msg.x = vec.x();
    point_msg.y = vec.y();
    point_msg.z = 0;
    return point_msg;
  }

  geometry_msgs::msg::Point packPoint(const Eigen::Vector3d& vec) {
    geometry_msgs::msg::Point point_msg = geometry_msgs::msg::Point();
    point_msg.x = vec.x();
    point_msg.y = vec.y();
    point_msg.z = vec.z();
    return point_msg;
  }

  geometry_msgs::msg::Transform packTransform(const Eigen::Affine3f& transform) {
    geometry_msgs::msg::Transform t;
    t.translation.x = transform.translation().x();
    t.translation.y = transform.translation().y();
    t.translation.z = transform.translation().z();

    Eigen::Quaternionf q(transform.linear());
    q.normalize();
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();
    t.rotation.w = q.w();

    return t;
  }

  geometry_msgs::msg::TransformStamped packTransformStamped(builtin_interfaces::msg::Time stamp,
                                                            std::string_view frame_id,
                                                            std::string_view child_frame_id,
                                                            const Eigen::Affine3f& transform) {

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.child_frame_id = child_frame_id;
    msg.transform = packTransform(transform);
    return msg;
  }
} // namespace nomadz_core
