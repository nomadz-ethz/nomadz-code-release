#pragma once

#include <rclcpp/rclcpp.hpp>

#include "nomadz_behavior/data_types/ego_status.hpp"
#include "nomadz_behavior/data_types/motion.hpp"
#include "nomadz_behavior/data_types/modeling.hpp"
#include "nomadz_behavior/data_types/teammates_status.hpp"
#include "nomadz_behavior/data_types/game_status.hpp"
#include "nomadz_behavior/data_types/proprioception.hpp"

#include "nomadz_behavior_msgs/msg/ego_status.hpp"
#include "nomadz_behavior_msgs/msg/game_status.hpp"
#include "nomadz_motion_control_msgs/msg/motion_request.hpp"
#include "nomadz_motion_control_msgs/msg/walk_request.hpp"
#include "nomadz_motion_control_msgs/msg/kick_request.hpp"
#include "nomadz_motion_control_msgs/msg/special_action_request.hpp"
#include "nomadz_motion_control_msgs/msg/head_motion_request.hpp"
#include "nomadz_modeling_msgs/msg/world_model.hpp"
#include "nomadz_modeling_msgs/msg/ball_model.hpp"
#include "nomadz_modeling_msgs/msg/robot_pose.hpp"

#include "nomadz_communication_msgs/msg/robocup_game_control_data.hpp"
#include "nomadz_communication_msgs/msg/team_info.hpp"
#include "nomadz_communication_msgs/msg/team_comm_info.hpp"

#include "nomadz_proprioception_msgs/msg/fall_down_state.hpp"
#include "nomadz_proprioception_msgs/msg/foot_support.hpp"

#include "nomadz_motion_control_msgs/msg/motion_info.hpp"

namespace nomadz_behavior {
  // change the conversion functions to toMsg and fromMsg
  // remove the suffix for explicit conversion. E.g. like in the yaml conversion.
  nomadz_behavior_msgs::msg::EgoStatus packEgoStatus(const EgoStatus& ego_status, const RobotPosture& posture);
  nomadz_motion_control_msgs::msg::WalkRequest packWalkRequest(const WalkRequest& walk_request);
  nomadz_motion_control_msgs::msg::KickRequest packKickRequest(const KickRequest& kick_request);
  nomadz_motion_control_msgs::msg::SpecialActionRequest
  packSpecialActionRequest(const SpecialActionRequest& special_action_request);
  nomadz_motion_control_msgs::msg::HeadMotionRequest packHeadMotionRequest(const HeadMotionRequest& head_motion_request);
  nomadz_motion_control_msgs::msg::MotionRequest packMotionRequest(const MotionRequest& motion_request,
                                                                   const HeadMotionRequest& head_motion_request,
                                                                   const rclcpp::Time& time_stamp);

  WalkRequest unpackWalkRequest(const nomadz_motion_control_msgs::msg::WalkRequest& walk_request_msg);
  KickRequest unpackKickRequest(const nomadz_motion_control_msgs::msg::KickRequest& kick_request_msg);
  SpecialActionRequest
  unpackSpecialActionRequest(const nomadz_motion_control_msgs::msg::SpecialActionRequest& special_action_request_msg);
  HeadMotionRequest
  unpackHeadMotionRequest(const nomadz_motion_control_msgs::msg::HeadMotionRequest& head_motion_request_msg);
  MotionRequest unpackMotionRequest(const nomadz_motion_control_msgs::msg::MotionRequest& motion_request_msg);
  MotionInfo unpackMotionInfo(const nomadz_motion_control_msgs::msg::MotionInfo& motion_info_msg);

  EgoStatus unpackEgoStatus(const nomadz_behavior_msgs::msg::EgoStatus& ego_status_msg);
  GameStatus unpackGameStatus(const nomadz_behavior_msgs::msg::GameStatus& game_status_msg);
  BallModel unpackBallModel(const nomadz_modeling_msgs::msg::BallModel& ball_model_msg);
  RobotPose unpackRobotPose(const nomadz_modeling_msgs::msg::RobotPose& robot_pose_msg);
  WorldModel unpackWorldModel(const nomadz_modeling_msgs::msg::WorldModel& world_model);

  BallModel computeTeamBallModel(const nomadz_communication_msgs::msg::TeamCommInfo& team_comm_info_msg,
                                 const rclcpp::Time& current_time_stamp,
                                 int player_id);
  CombinedWorldModel unpackCombinedWorldModel(const nomadz_modeling_msgs::msg::WorldModel& world_model,
                                              const nomadz_communication_msgs::msg::TeamCommInfo& team_comm_info_msg,
                                              const rclcpp::Time& current_time_stamp,
                                              int player_id);

  RobotPosture unpackRobotPosture(const nomadz_proprioception_msgs::msg::FallDownState& fall_down_state_msg,
                                  const nomadz_proprioception_msgs::msg::FootSupport& foot_support_msg);
  TeammatesStatus unpackTeammateStatus(const nomadz_communication_msgs::msg::TeamCommInfo& team_comm_info_msg,
                                       const nomadz_communication_msgs::msg::TeamInfo& own_team_info_msg);
} // namespace nomadz_behavior
