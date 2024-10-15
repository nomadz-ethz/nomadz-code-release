#pragma once

#include <rclcpp/rclcpp.hpp>

#include "nomadz_kinematics/robot_model.hpp"
#include "nomadz_proprioception_msgs/msg/robot_model.hpp"

namespace nomadz_kinematics {
  nomadz_proprioception_msgs::msg::RobotModel packRobotModel(const RobotModel& robot_model, const rclcpp::Time& time_stamp);
} // namespace nomadz_kinematics
