#include "nomadz_communication/ros_conversion.hpp"

#include <algorithm>

namespace nomadz_communication {
  nomadz_communication_msgs::msg::RobocupGameControlData packRobocupGameControlData(const RobocupGameControlData& data,
                                                                                    const rclcpp::Time& time_stamp) {
    nomadz_communication_msgs::msg::RobocupGameControlData msg;
    msg.header.stamp = time_stamp;
    msg.packet_number = data.packet_number;
    msg.players_per_team = data.players_per_team;
    msg.competition_phase = static_cast<uint8_t>(data.competition_phase);
    msg.competition_type = static_cast<uint8_t>(data.competition_type);
    msg.game_phase = static_cast<uint8_t>(data.game_phase);
    msg.state = static_cast<uint8_t>(data.state);
    msg.set_play = static_cast<uint8_t>(data.set_play);
    msg.first_half = data.first_half;
    msg.kicking_team = data.kicking_team;
    msg.secs_remaining = data.secs_remaining;
    msg.secondary_time = data.secondary_time;

    for (int i = 0; i < 2; i++) {
      msg.teams[i].team_id = data.teams[i].team_id;
      msg.teams[i].field_player_colour = static_cast<uint8_t>(data.teams[i].field_player_colour);
      msg.teams[i].goal_keeper_colour = static_cast<uint8_t>(data.teams[i].goal_keeper_colour);
      msg.teams[i].goal_keeper = data.teams[i].goal_keeper;
      msg.teams[i].score = data.teams[i].score;
      msg.teams[i].penalty_shot = data.teams[i].penalty_shot;
      msg.teams[i].single_shots = data.teams[i].single_shots;
      msg.teams[i].message_budget = data.teams[i].message_budget;

      for (int j = 0; j < MAX_NUM_PLAYERS; j++) {
        msg.teams[i].players[j].penalty = static_cast<uint8_t>(data.teams[i].players[j].penalty);
        msg.teams[i].players[j].secs_till_unpenalised = data.teams[i].players[j].secs_till_unpenalised;
      }
    }

    return msg;
  }

  RobocupGameControlReturnData
  unpackRobocupGameControlReturnData(const nomadz_communication_msgs::msg::RobocupGameControlReturnData& msg) {
    RobocupGameControlReturnData data;
    std::string header = GAMECONTROLLER_RETURN_STRUCT_HEADER;
    header.copy(data.header, header.size());
    data.version = GAMECONTROLLER_RETURN_STRUCT_VERSION;
    data.player_num = msg.player_num;
    data.team_num = msg.team_num;
    data.has_fallen = msg.has_fallen;
    data.pose[0] = static_cast<float>(msg.pose.x);
    data.pose[1] = static_cast<float>(msg.pose.y);
    data.pose[2] = static_cast<float>(msg.pose.theta);
    data.ball_age = msg.ball_age;
    data.ball[0] = static_cast<float>(msg.ball.x);
    data.ball[1] = static_cast<float>(msg.ball.y);

    return data;
  }

  nomadz_behavior_msgs::msg::EgoStatus packEgoStatusCompact(const EgoStatusCompact& data) {
    nomadz_behavior_msgs::msg::EgoStatus msg;
    msg.whistle_detected = data.whistle_detected;
    msg.has_fallen = data.has_fallen;
    msg.want_ball = data.want_ball;
    msg.has_ball_lock = data.has_ball_lock;
    msg.ball_score = data.ball_score;
    return msg;
  }

  EgoStatusCompact unpackEgoStatusCompact(const nomadz_behavior_msgs::msg::EgoStatus& msg) {
    EgoStatusCompact ego_status{};
    ego_status.whistle_detected = msg.whistle_detected;
    ego_status.has_fallen = msg.has_fallen;
    ego_status.want_ball = msg.want_ball;
    ego_status.has_ball_lock = msg.has_ball_lock;
    ego_status.ball_score = msg.ball_score;
    return ego_status;
  }

  nomadz_modeling_msgs::msg::BallModel packBallModelCompact(const BallModelCompact& data) {
    nomadz_modeling_msgs::msg::BallModel msg;
    msg.position.x = static_cast<double>(data.x) / TO_UINT8_SCALER_X;
    msg.position.y = static_cast<double>(data.y) / TO_UINT8_SCALER_Y;
    msg.valid = data.valid;
    msg.lost = !data.valid;
    return msg;
  }

  BallModelCompact unpackBallModelCompact(const nomadz_modeling_msgs::msg::BallModel& msg) {
    BallModelCompact ball_model{};
    ball_model.x =
      static_cast<int8_t>(std::clamp(static_cast<float>(msg.position.x), -MAX_X_POS, MAX_X_POS) * TO_UINT8_SCALER_X);
    ball_model.y =
      static_cast<int8_t>(std::clamp(static_cast<float>(msg.position.y), -MAX_Y_POS, MAX_Y_POS) * TO_UINT8_SCALER_Y);
    ball_model.valid = !msg.lost;
    return ball_model;
  }

  nomadz_modeling_msgs::msg::RobotPose packRobotPoseCompact(const RobotPoseCompact& data) {
    nomadz_modeling_msgs::msg::RobotPose msg;
    msg.pose.x = static_cast<double>(data.x) / TO_UINT8_SCALER_X;
    msg.pose.y = static_cast<double>(data.y) / TO_UINT8_SCALER_Y;
    msg.pose.theta = static_cast<double>(data.theta) / TO_UINT8_SCALER_THETA;
    msg.valid = data.valid;
    msg.lost = !data.valid;
    return msg;
  }

  RobotPoseCompact unpackRobotPoseCompact(const nomadz_modeling_msgs::msg::RobotPose& msg) {
    RobotPoseCompact robot_pose{};
    robot_pose.x =
      static_cast<int8_t>(std::clamp(static_cast<float>(msg.pose.x), -MAX_X_POS, MAX_X_POS) * TO_UINT8_SCALER_X);
    robot_pose.y =
      static_cast<int8_t>(std::clamp(static_cast<float>(msg.pose.y), -MAX_Y_POS, MAX_Y_POS) * TO_UINT8_SCALER_Y);
    robot_pose.theta = static_cast<int8_t>(msg.pose.theta * TO_UINT8_SCALER_THETA);
    robot_pose.valid = !msg.lost;
    return robot_pose;
  }

  nomadz_communication_msgs::msg::TeamCommData packTeamCommData(const TeamCommData& data, const rclcpp::Time& time_stamp) {
    nomadz_communication_msgs::msg::TeamCommData msg;
    msg.header.stamp = time_stamp;
    msg.player_id = data.player_id;

    msg.ego_status = packEgoStatusCompact(data.ego_status);
    msg.teammate_ball_model = packBallModelCompact(data.ball_model);
    msg.teammate_pose = packRobotPoseCompact(data.robot_pose);

    return msg;
  }

  TeamCommData unpackTeamCommData(const int player_id,
                                  const nomadz_behavior_msgs::msg::EgoStatus& ego_status_msg,
                                  const nomadz_modeling_msgs::msg::WorldModel& world_model_msg) {
    TeamCommData team_comm_data{};
    team_comm_data.player_id = player_id;

    team_comm_data.ego_status = unpackEgoStatusCompact(ego_status_msg);
    team_comm_data.ball_model = unpackBallModelCompact(world_model_msg.ball_model);
    team_comm_data.robot_pose = unpackRobotPoseCompact(world_model_msg.robot_pose);

    return team_comm_data;
  }
} // namespace nomadz_communication
