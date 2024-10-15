#pragma once

#include <rclcpp/rclcpp.hpp>

#include "nomadz_core/math/constants.hpp"
#include "nomadz_communication/robocup_game_control_data.hpp"
#include "nomadz_communication/robocup_game_control_return_data.hpp"
#include "nomadz_communication/team_comm_data.hpp"
#include "nomadz_communication_msgs/msg/robocup_game_control_data.hpp"
#include "nomadz_communication_msgs/msg/robocup_game_control_return_data.hpp"
#include "nomadz_communication_msgs/msg/team_comm_data.hpp"
#include "nomadz_behavior_msgs/msg/ego_status.hpp"
#include "nomadz_modeling_msgs/msg/ball_model.hpp"
#include "nomadz_modeling_msgs/msg/robot_pose.hpp"
#include "nomadz_modeling_msgs/msg/world_model.hpp"

namespace nomadz_communication {
  constexpr float MAX_X_POS = 5.5F; // Max represented field x position [-5.5m, 5.5m]
  constexpr float MAX_Y_POS = 4.F;  // Max represented field y position [-4m, 4m]
  // for symmetry reason we are not using the -128 value of the uint8
  constexpr float TO_UINT8_SCALER_X = 127.F / MAX_X_POS;
  constexpr float TO_UINT8_SCALER_Y = 127.F / MAX_Y_POS;
  constexpr float TO_UINT8_SCALER_THETA = 127.F / nomadz_core::constants::PI;

  nomadz_communication_msgs::msg::RobocupGameControlData packRobocupGameControlData(const RobocupGameControlData& data,
                                                                                    const rclcpp::Time& time_stamp);

  RobocupGameControlReturnData
  unpackRobocupGameControlReturnData(const nomadz_communication_msgs::msg::RobocupGameControlReturnData& msg);

  nomadz_behavior_msgs::msg::EgoStatus packEgoStatusCompact(const EgoStatusCompact& data);

  EgoStatusCompact unpackEgoStatusCompact(const nomadz_behavior_msgs::msg::EgoStatus& msg);

  nomadz_modeling_msgs::msg::BallModel packBallModelCompact(const BallModelCompact& data);

  BallModelCompact unpackBallModelCompact(const nomadz_modeling_msgs::msg::BallModel& msg);

  nomadz_modeling_msgs::msg::RobotPose packRobotPoseCompact(const RobotPoseCompact& data);

  RobotPoseCompact unpackRobotPoseCompact(const nomadz_modeling_msgs::msg::RobotPose& msg);

  nomadz_communication_msgs::msg::TeamCommData packTeamCommData(const TeamCommData& data, const rclcpp::Time& time_stamp);

  TeamCommData unpackTeamCommData(int player_id,
                                  const nomadz_behavior_msgs::msg::EgoStatus& ego_status_msg,
                                  const nomadz_modeling_msgs::msg::WorldModel& world_model_msg);
} // namespace nomadz_communication
