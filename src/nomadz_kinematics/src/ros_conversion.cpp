#include "nomadz_kinematics/ros_conversion.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include "nomadz_core/geometry/ros_conversion.hpp"

namespace limbs = nomadz_definitions::limbs;
namespace side = nomadz_definitions::side;

namespace nomadz_kinematics {
  nomadz_proprioception_msgs::msg::RobotModel packRobotModel(const RobotModel& robot_model, const rclcpp::Time& time_stamp) {
    nomadz_proprioception_msgs::msg::RobotModel msg;
    msg.header.stamp = time_stamp;
    msg.inertia.com = nomadz_core::packVector3(robot_model.getCenterOfMass());

    msg.inertia.m = RobotModel::getTotalMass();

    msg.inertia.ixx = robot_model.getInertiaMatrix()(0, 0);
    msg.inertia.ixy = robot_model.getInertiaMatrix()(0, 1);
    msg.inertia.ixz = robot_model.getInertiaMatrix()(0, 2);
    msg.inertia.iyy = robot_model.getInertiaMatrix()(1, 1);
    msg.inertia.iyz = robot_model.getInertiaMatrix()(1, 2);
    msg.inertia.izz = robot_model.getInertiaMatrix()(2, 2);

    msg.head_pose = tf2::toMsg(robot_model.getLimbs()[limbs::HEAD].cast<double>());
    msg.hand_poses[side::LEFT] = tf2::toMsg(robot_model.getLimbs()[limbs::WRIST_LEFT].cast<double>());
    msg.hand_poses[side::RIGHT] = tf2::toMsg(robot_model.getLimbs()[limbs::WRIST_RIGHT].cast<double>());
    msg.foot_poses[side::LEFT] = tf2::toMsg(robot_model.getLimbs()[limbs::FOOT_LEFT].cast<double>());
    msg.foot_poses[side::RIGHT] = tf2::toMsg(robot_model.getLimbs()[limbs::FOOT_RIGHT].cast<double>());
    return msg;
  }
} // namespace nomadz_kinematics
