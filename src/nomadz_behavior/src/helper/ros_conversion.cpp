#include "nomadz_behavior/helper/ros_conversion.hpp"

#include "nomadz_core/geometry/ros_conversion.hpp"
#include "nomadz_proprioception_msgs/fall_down_state_enums.hpp"
#include "nomadz_communication_msgs/robocup_game_control_data_enums.hpp"

namespace nomadz_behavior {
  nomadz_behavior_msgs::msg::EgoStatus packEgoStatus(const EgoStatus& ego_status, const RobotPosture& posture) {
    nomadz_behavior_msgs::msg::EgoStatus msg;
    msg.whistle_detected = ego_status.whistle_detected;
    msg.is_penalized = ego_status.is_penalized;
    msg.has_fallen = posture.is_on_ground;
    msg.want_ball = ego_status.want_ball;
    msg.has_ball_lock = ego_status.has_ball_lock;
    msg.ball_score = ego_status.ball_score;
    return msg;
  }

  nomadz_motion_control_msgs::msg::WalkRequest packWalkRequest(const WalkRequest& walk_request) {
    nomadz_motion_control_msgs::msg::WalkRequest msg;
    msg.walk_mode = static_cast<uint8_t>(walk_request.mode);
    msg.target_speed = nomadz_core::packTwist2D(walk_request.target_speed);
    msg.pattern_type = static_cast<uint8_t>(walk_request.pattern_type);
    return msg;
  }

  nomadz_motion_control_msgs::msg::KickRequest packKickRequest(const KickRequest& kick_request) {
    nomadz_motion_control_msgs::msg::KickRequest msg;
    msg.kick_type = static_cast<uint8_t>(kick_request.kick_type);
    msg.mirror = kick_request.mirror;
    msg.kick_power = kick_request.kick_power;
    msg.kick_direction = kick_request.kick_direction;
    return msg;
  }

  nomadz_motion_control_msgs::msg::SpecialActionRequest
  packSpecialActionRequest(const SpecialActionRequest& special_action_request) {
    nomadz_motion_control_msgs::msg::SpecialActionRequest msg;
    msg.special_action_type = static_cast<uint8_t>(special_action_request.special_action_type);
    msg.mirror = special_action_request.mirror;
    return msg;
  }

  nomadz_motion_control_msgs::msg::HeadMotionRequest packHeadMotionRequest(const HeadMotionRequest& head_motion_request) {
    nomadz_motion_control_msgs::msg::HeadMotionRequest msg;
    msg.head_motion_type = static_cast<uint8_t>(head_motion_request.head_motion_type);
    msg.pan = head_motion_request.pan;
    msg.tilt = head_motion_request.tilt;
    msg.speed = 0.2F;
    msg.target = nomadz_core::packVector3(head_motion_request.target);
    return msg;
  }

  nomadz_motion_control_msgs::msg::MotionRequest packMotionRequest(const MotionRequest& motion_request,
                                                                   const HeadMotionRequest& head_motion_request,
                                                                   const rclcpp::Time& time_stamp) {
    nomadz_motion_control_msgs::msg::MotionRequest msg;
    msg.header.stamp = time_stamp;
    msg.motion_type = static_cast<uint8_t>(motion_request.motion_type);
    msg.walk_request = packWalkRequest(motion_request.walk_request);
    msg.kick_request = packKickRequest(motion_request.kick_request);
    msg.special_action_request = packSpecialActionRequest(motion_request.special_action_request);
    msg.head_motion_request = packHeadMotionRequest(head_motion_request);
    return msg;
  }

  WalkRequest unpackWalkRequest(const nomadz_motion_control_msgs::msg::WalkRequest& walk_request_msg) {
    WalkRequest walk_request;
    walk_request.mode = static_cast<WalkMode>(walk_request_msg.walk_mode);
    walk_request.target_speed = nomadz_core::unpackTwist2D(walk_request_msg.target_speed);
    walk_request.pattern_type = static_cast<PatternType>(walk_request_msg.pattern_type);
    return walk_request;
  }

  KickRequest unpackKickRequest(const nomadz_motion_control_msgs::msg::KickRequest& kick_request_msg) {
    KickRequest kick_request;
    kick_request.kick_type = static_cast<KickType>(kick_request_msg.kick_type);
    kick_request.mirror = kick_request_msg.mirror;
    kick_request.kick_power = kick_request_msg.kick_power;
    kick_request.kick_direction = kick_request_msg.kick_direction;
    return kick_request;
  }

  SpecialActionRequest
  unpackSpecialActionRequest(const nomadz_motion_control_msgs::msg::SpecialActionRequest& special_action_request_msg) {
    SpecialActionRequest special_action_request;
    special_action_request.special_action_type =
      static_cast<SpecialActionType>(special_action_request_msg.special_action_type);
    special_action_request.mirror = special_action_request_msg.mirror;
    return special_action_request;
  }

  HeadMotionRequest
  unpackHeadMotionRequest(const nomadz_motion_control_msgs::msg::HeadMotionRequest& head_motion_request_msg) {
    HeadMotionRequest head_motion_request;
    head_motion_request.head_motion_type = static_cast<HeadMotionType>(head_motion_request_msg.head_motion_type);
    head_motion_request.pan = head_motion_request_msg.pan;
    head_motion_request.tilt = head_motion_request_msg.tilt;
    head_motion_request.target = nomadz_core::unpackVector3(head_motion_request_msg.target);
    return head_motion_request;
  }

  MotionRequest unpackMotionRequest(const nomadz_motion_control_msgs::msg::MotionRequest& motion_request_msg) {
    MotionRequest motion_request;
    motion_request.motion_type = static_cast<MotionType>(motion_request_msg.motion_type);
    motion_request.walk_request = unpackWalkRequest(motion_request_msg.walk_request);
    motion_request.kick_request = unpackKickRequest(motion_request_msg.kick_request);
    motion_request.special_action_request = unpackSpecialActionRequest(motion_request_msg.special_action_request);
    return motion_request;
  }

  MotionInfo unpackMotionInfo(const nomadz_motion_control_msgs::msg::MotionInfo& motion_info_msg) {
    MotionInfo motion_info;
    motion_info.executed_motion_request = unpackMotionRequest(motion_info_msg.executed_motion_request);
    motion_info.executed_head_motion_request =
      unpackHeadMotionRequest(motion_info_msg.executed_motion_request.head_motion_request);
    motion_info.is_motion_done = motion_info_msg.is_motion_done;
    motion_info.is_head_motion_done = motion_info_msg.is_head_motion_done;
    return motion_info;
  }

  EgoStatus unpackEgoStatus(const nomadz_behavior_msgs::msg::EgoStatus& ego_status_msg) {
    EgoStatus ego_status;
    ego_status.whistle_detected = ego_status_msg.whistle_detected;
    ego_status.is_penalized = ego_status_msg.is_penalized;
    ego_status.has_fallen = ego_status_msg.has_fallen;
    ego_status.want_ball = ego_status_msg.want_ball;
    ego_status.has_ball_lock = ego_status_msg.has_ball_lock;
    ego_status.ball_score = ego_status_msg.ball_score;
    return ego_status;
  }

  GameStatus unpackGameStatus(const nomadz_behavior_msgs::msg::GameStatus& game_status_msg) {
    GameStatus game_status;
    game_status.game_phase = static_cast<GameStatus::GamePhase>(game_status_msg.game_phase);
    game_status.game_state = static_cast<GameStatus::GameState>(game_status_msg.game_state);
    game_status.set_play = static_cast<GameStatus::SetPlay>(game_status_msg.set_play);
    game_status.is_first_half = game_status_msg.is_first_half;
    game_status.is_kicking_team = game_status_msg.is_kicking_team;
    return game_status;
  }

  BallModel unpackBallModel(const nomadz_modeling_msgs::msg::BallModel& ball_model_msg) {
    BallModel ball_model;
    ball_model.position.x() = ball_model_msg.position.x;
    ball_model.position.y() = ball_model_msg.position.y;
    ball_model.velocity.x() = ball_model_msg.velocity.x;
    ball_model.velocity.y() = ball_model_msg.velocity.y;
    ball_model.valid = ball_model_msg.valid;
    ball_model.lost = ball_model_msg.lost;
    return ball_model;
  }

  RobotPose unpackRobotPose(const nomadz_modeling_msgs::msg::RobotPose& robot_pose_msg) {
    RobotPose robot_pose;
    robot_pose.pose.x = robot_pose_msg.pose.x;
    robot_pose.pose.y = robot_pose_msg.pose.y;
    robot_pose.pose.theta = robot_pose_msg.pose.theta;
    robot_pose.valid = robot_pose_msg.valid;
    robot_pose.lost = robot_pose_msg.lost;
    return robot_pose;
  }

  WorldModel unpackWorldModel(const nomadz_modeling_msgs::msg::WorldModel& world_model_msg) {
    WorldModel world_model;
    world_model.ball_model = unpackBallModel(world_model_msg.ball_model);
    world_model.robot_pose = unpackRobotPose(world_model_msg.robot_pose);
    for (const auto& teammate_pose : world_model_msg.teammate_poses) {
      world_model.teammate_poses.push_back(nomadz_core::unpackPose2D(teammate_pose));
    }
    for (const auto& opponent_pose : world_model_msg.opponent_poses) {
      world_model.opponent_poses.push_back(nomadz_core::unpackPose2D(opponent_pose));
    }
    return world_model;
  }

  BallModel computeTeamBallModel(const nomadz_communication_msgs::msg::TeamCommInfo& team_comm_info_msg,
                                 const rclcpp::Time& current_time_stamp,
                                 const int player_id) {
    BallModel team_ball_model;
    float accumulated_weight = 0.F;
    float most_recent_ball_update = 1000.F;
    for (int i = 0; i < MAX_NUM_OF_PLAYERS; i++) {
      if (team_comm_info_msg.team_comm_info[i].teammate_ball_model.lost || i == player_id) {
        continue;
      }
      const float time_since_update = static_cast<float>(
        static_cast<rclcpp::Duration>(current_time_stamp - team_comm_info_msg.team_comm_info[i].header.stamp).seconds());
      most_recent_ball_update = std::min(most_recent_ball_update, time_since_update);
      const float weight_factor = std::max(0.F, 1.F - time_since_update / 10.F);
      accumulated_weight += weight_factor;
      team_ball_model.position.x() +=
        weight_factor * static_cast<float>(team_comm_info_msg.team_comm_info[i].teammate_ball_model.position.x);
      team_ball_model.position.y() +=
        weight_factor * static_cast<float>(team_comm_info_msg.team_comm_info[i].teammate_ball_model.position.y);
    }
    team_ball_model.position /= accumulated_weight;
    team_ball_model.valid = most_recent_ball_update > 5.F;
    return team_ball_model;
  }

  CombinedWorldModel unpackCombinedWorldModel(const nomadz_modeling_msgs::msg::WorldModel& world_model_msg,
                                              const nomadz_communication_msgs::msg::TeamCommInfo& team_comm_info_msg,
                                              const rclcpp::Time& current_time_stamp,
                                              const int player_id) {
    CombinedWorldModel combined_world_model;
    combined_world_model.team_ball_model = computeTeamBallModel(team_comm_info_msg, current_time_stamp, player_id);
    for (int i = 0; i < MAX_NUM_OF_PLAYERS; i++) {
      combined_world_model.teammate_poses[i] =
        nomadz_core::unpackPose2D(team_comm_info_msg.team_comm_info[i].teammate_pose.pose);
    }
    int robot_index = 0;
    for (const auto& opponent_pose : world_model_msg.opponent_poses) {
      combined_world_model.opponent_poses[robot_index] = nomadz_core::unpackPose2D(opponent_pose);
      robot_index++;
    }
    return combined_world_model;
  }

  RobotPosture unpackRobotPosture(const nomadz_proprioception_msgs::msg::FallDownState& fall_down_state_msg,
                                  const nomadz_proprioception_msgs::msg::FootSupport& foot_support_msg) {
    RobotPosture robot_posture;
    robot_posture.is_standing =
      fall_down_state_msg.fall_down_state == static_cast<uint8_t>(nomadz_proprioception_msgs::FallDownState::UPRIGHT);
    robot_posture.is_on_ground =
      fall_down_state_msg.fall_down_state == static_cast<uint8_t>(nomadz_proprioception_msgs::FallDownState::ON_GROUND);
    robot_posture.has_ground_contact = foot_support_msg.ground_contacts[0] && foot_support_msg.ground_contacts[1];
    robot_posture.fall_direction = static_cast<RobotPosture::FallDirection>(fall_down_state_msg.fall_direction);
    return robot_posture;
  }

  TeammatesStatus unpackTeammateStatus(const nomadz_communication_msgs::msg::TeamCommInfo& team_comm_info_msg,
                                       const nomadz_communication_msgs::msg::TeamInfo& own_team_info_msg) {
    TeammatesStatus teammates_status;
    for (int i = 0; i < MAX_NUM_OF_PLAYERS; i++) {

      teammates_status.role[i] = TeammatesStatus::Role::Player;
      teammates_status.teammate_ego_status[i] = unpackEgoStatus(team_comm_info_msg.team_comm_info[i].ego_status);
      teammates_status.is_active[i] =
        own_team_info_msg.players[i].penalty == static_cast<uint8_t>(nomadz_communication_msgs::Penalty::NONE);
    }

    return teammates_status;
  }

} // namespace nomadz_behavior
