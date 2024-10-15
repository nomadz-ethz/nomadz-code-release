#include "nomadz_behavior/helper/target_pose_generator.hpp"
#include "nomadz_behavior/helper/common_functions.hpp"

#include <spdlog/spdlog.h>
#include <cmath>

using Pose2D = nomadz_core::Pose2D;
using nomadz_core::constants::PI;

namespace nomadz_behavior {

  TargetPoseGenerator::TargetPoseGenerator(BT::Blackboard::Ptr blackboard) : blackboard_(std::move(blackboard)) {}

  bool TargetPoseGenerator::getTargetPose(const TargetType request, Pose2D& target_pose) {
    spdlog::info("getTargetPose");
    opt_variables_ = loadTargetPoseGeneratorParameters();

    world_model_ = blackboard_->get<WorldModel>("world_model");
    combined_world_model_ = blackboard_->get<CombinedWorldModel>("combined_world_model");
    field_lines_ = blackboard_->get<FieldLines>("field_lines");
    teammates_status_ = blackboard_->get<TeammatesStatus>("teammates_status");
    player_id_ = blackboard_->get<GameSettings>("game_settings").player_id;

    switch (request) {
    case TargetType::GAME_READY:
      target_pose = getReadyPose();
      break;
    case TargetType::CENTER_GOAL:
      target_pose = getCenterGoalPose();
      break;
    case TargetType::DETECT_SHOT:
      target_pose = getDetectShotPose();
      break;
    case TargetType::SEARCH:
      target_pose = getOptimalSearchPose();
      break;
    case TargetType::COVER:
      target_pose = getOptimalCoverPose();
      break;
    case TargetType::OBSERVATION:
      target_pose = getObservationPose();
      break;
    case TargetType::ENGAGE:
      target_pose = getEngagePose();
      break;
    case TargetType::PENALTY_KICK:
      target_pose = getPenaltyKickPose();
      break;

    default:
      spdlog::warn("getTargetPose: target_type not valid");
      return false;
    }
    return true;
  }

  Pose2D TargetPoseGenerator::getReadyPose() const {
    Pose2D normalized_ready_pose;

    bool is_kicking_team = blackboard_->get<GameStatus>("game_status").is_kicking_team;

    int other_active_players = getNumberLowerActiveFieldPlayers(player_id_ - 1, teammates_status_.is_active);

    if (is_kicking_team) {
      normalized_ready_pose = nomadz_core::toPose2D(opt_variables_.ready_pose_kicking_team.at(other_active_players));
    } else {
      normalized_ready_pose = nomadz_core::toPose2D(opt_variables_.ready_pose_non_kicking_team.at(other_active_players));
    }

    return denormalizePose(normalized_ready_pose);
  }

  Pose2D TargetPoseGenerator::denormalizePose(const Pose2D& pose) const {
    return Pose2D{pose.x * field_lines_.x_pos_opponent_ground_line, pose.y * field_lines_.y_pos_left_sideline, pose.theta};
  }

  Eigen::Vector2f TargetPoseGenerator::getBallPosition() const {
    if (world_model_.ball_model.valid) {
      return globalFromLocal(world_model_.ball_model.position, world_model_.robot_pose.pose);
    }

    return combined_world_model_.team_ball_model.position;
  }

  Pose2D TargetPoseGenerator::getCenterGoalPose() const {
    // same as ready pose non-kicking team
    return denormalizePose(nomadz_core::toPose2D(opt_variables_.ready_pose_non_kicking_team.at(0)));
  }

  Pose2D TargetPoseGenerator::getDetectShotPose() const {
    Eigen::Vector2f center_goal = {field_lines_.x_pos_own_goal_post, 0.F};
    Eigen::Vector2f center_goal_to_ball = (getBallPosition() - center_goal).normalized();
    Eigen::Vector2f detect_shot_position = center_goal + opt_variables_.des_distance_goalkeeper_cover * center_goal_to_ball;

    // todo: not ideal if ball close/ on/ behin goal

    return Pose2D{
      detect_shot_position[0], detect_shot_position[1], std::atan2(center_goal_to_ball[1], center_goal_to_ball[0])};
  }

  Pose2D TargetPoseGenerator::getPenaltyKickPose() const {
    const Eigen::Vector2f penalty_mark = {field_lines_.x_pos_opponent_penalty_mark, 0.F};
    const Eigen::Vector2f target_goal_shot = {field_lines_.x_pos_opponent_goal_post,
                                              field_lines_.y_pos_left_goal - opt_variables_.distance_penalty_kick_goalpost};

    const Eigen::Vector2f penalty_mark_to_target_shot = target_goal_shot - penalty_mark;
    const Eigen::Vector2f target_penalty_kick =
      penalty_mark - opt_variables_.distance_to_ball * penalty_mark_to_target_shot.normalized();

    return Pose2D{target_penalty_kick[0],
                  target_penalty_kick[1],
                  std::atan2(penalty_mark_to_target_shot[1], penalty_mark_to_target_shot[0])};
  }

  Pose2D TargetPoseGenerator::getPoseDistanceToBall(const float desired_distance) const {
    Eigen::Vector2f ball_position = getBallPosition();
    Eigen::Vector2f robot_position = {world_model_.robot_pose.pose.x, world_model_.robot_pose.pose.y};

    Eigen::Vector2f robot_to_ball = ball_position - robot_position;
    float distance = robot_to_ball.norm();

    Eigen::Vector2f target_position = robot_position + (distance - desired_distance) / distance * robot_to_ball;

    Pose2D target_pose;
    target_pose.x = target_position[0];
    target_pose.y = target_position[1];

    target_pose.theta = std::atan2(robot_to_ball[1], robot_to_ball[0]);

    return target_pose;
  }

  Pose2D TargetPoseGenerator::getObservationPose() const {
    return getPoseDistanceToBall(opt_variables_.distance_observation_to_ball);
  }

  Pose2D TargetPoseGenerator::getEngagePose() const {
    return getPoseDistanceToBall(opt_variables_.distance_to_ball);
  }

  Pose2D TargetPoseGenerator::getOptimalCoverPose() const {
    return getOptimalPose(false);
  }

  Pose2D TargetPoseGenerator::getOptimalSearchPose() const {
    return getOptimalPose(true);
  }

  Pose2D TargetPoseGenerator::getOptimalPose(const bool ball_lost) const {
    std::vector<float> robot_pose = {world_model_.robot_pose.pose.x, world_model_.robot_pose.pose.y};

    std::vector<float> costs(9);     // cost list: C, N, NE, E, SE, S, SW, W, NW
    std::vector<float> gradients(8); // no center
    std::vector<float> queried_pos(2);

    for (int i = 0; i < 9; i++) {
      switch (i) {
      case 0:
        queried_pos = robot_pose;
        break;
      case 1:
        queried_pos = {robot_pose[0] + opt_variables_.step_size_gradient, robot_pose[1]};
        break;
      case 2:
        queried_pos = {robot_pose[0] + opt_variables_.step_size_gradient, robot_pose[1] - opt_variables_.step_size_gradient};
        break;
      case 3:
        queried_pos = {robot_pose[0], robot_pose[1] - opt_variables_.step_size_gradient};
        break;
      case 4:
        queried_pos = {robot_pose[0] - opt_variables_.step_size_gradient, robot_pose[1] - opt_variables_.step_size_gradient};
        break;
      case 5:
        queried_pos = {robot_pose[0] - opt_variables_.step_size_gradient, robot_pose[1]};
        break;
      case 6:
        queried_pos = {robot_pose[0] - opt_variables_.step_size_gradient, robot_pose[1] + opt_variables_.step_size_gradient};
        break;
      case 7:
        queried_pos = {robot_pose[0], robot_pose[1] + opt_variables_.step_size_gradient};
        break;
      case 8:
        queried_pos = {robot_pose[0] + opt_variables_.step_size_gradient, robot_pose[1] + opt_variables_.step_size_gradient};
        break;
      }

      if (ball_lost) {
        costs[i] = getCostSearch(queried_pos);
      } else {
        costs[i] = getCostPlay(queried_pos);
      }
    }

    for (int i = 0; i < 8; i++) {
      gradients[i] = costs[i + 1] - costs[0];
    }

    const auto iter_min_gradient = std::min_element(gradients.begin(), gradients.end()); // might be negative
    const float min_gradient = *iter_min_gradient;
    const auto optimal_direction = std::distance(gradients.begin(), iter_min_gradient);

    const float translation = opt_variables_.scale_gradient * abs(min_gradient);

    Pose2D optimal_pose;

    if (ball_lost) {
      optimal_pose.theta = -static_cast<float>(optimal_direction) * PI / 4.F;
    } else {
      Eigen::Vector2f ball_position = getBallPosition();
      optimal_pose.theta =
        std::atan2(ball_position[1] - world_model_.robot_pose.pose.y, ball_position[0] - world_model_.robot_pose.pose.x);
    }

    switch (optimal_direction) {
    case (0):
      optimal_pose.x = robot_pose[0] + translation;
      optimal_pose.y = robot_pose[1];
      break;
    case (1):
      optimal_pose.x = robot_pose[0] + translation;
      optimal_pose.y = robot_pose[1] - translation;
      break;
    case (2):
      optimal_pose.x = robot_pose[0];
      optimal_pose.y = robot_pose[1] - translation;
      break;
    case (3):
      optimal_pose.x = robot_pose[0] - translation;
      optimal_pose.y = robot_pose[1] - translation;
      break;
    case (4):
      optimal_pose.x = robot_pose[0] - translation;
      optimal_pose.y = robot_pose[1];
      break;
    case (5):
      optimal_pose.x = robot_pose[0] - translation;
      optimal_pose.y = robot_pose[1] + translation;
      break;
    case (6):
      optimal_pose.x = robot_pose[0];
      optimal_pose.y = robot_pose[1] + translation;
      break;
    case (7):
      optimal_pose.x = robot_pose[0] + translation;
      optimal_pose.y = robot_pose[1] + translation;
      break;
    }

    // project into some smaller area
    optimal_pose.x =
      std::min(std::max(optimal_pose.x, field_lines_.x_pos_own_goal_area), field_lines_.x_pos_opponent_goal_area);
    optimal_pose.y =
      std::min(std::max(optimal_pose.y, field_lines_.y_pos_left_sideline), field_lines_.y_pos_right_penalty_area);

    return optimal_pose;
  }

  float TargetPoseGenerator::getCostSearch(const std::vector<float>& position) const {
    return getCoverCost(position) +
           // not included for now since we don't really have a good estimate
           //  getBallCoMCost(position,
           //                 {ball_position[0],
           //                  ball_position[1]}) +
           getAvoidanceCost(position);
  }

  float TargetPoseGenerator::getCostPlay(const std::vector<float>& position) const {
    Eigen::Vector2f ball_position = getBallPosition();
    return getCoverCost(position) +
           // we track 1.5m ahead of the ball, since the robot with ballock is there already
           getBallCoMCost(position, {ball_position[0] + 1500, ball_position[1]}) + getAvoidanceCost(position) +
           getGoalFreeCost(position) + getOffenseDefenseCost(position);
  }

  float TargetPoseGenerator::getCoverCost(const std::vector<float>& position) const {
    if (std::abs(position[0]) > field_lines_.x_pos_opponent_ground_line ||
        std::abs(position[1]) > field_lines_.y_pos_left_sideline) {
      return 0.F;
    }

    float value = 0.F;
    float error1;
    float error2;

    for (int i = 0; i < MAX_NUM_OF_PLAYERS; i++) {
      if (i == (player_id_ - 1) || !teammates_status_.is_active.at(i)) {
        continue;
      }

      for (float dx = -1.F; dx < 1.1F; dx += 0.2F) {
        // no reward for outside the field, for robustness within slightly smaller field

        if (position[0] + dx > field_lines_.x_pos_opponent_goal_area ||
            position[0] + dx < field_lines_.x_pos_own_goal_area) {
          continue;
        }

        for (float dy = -1.F; dy < 1.1F; dy += 0.2F) {
          if (std::abs(position[1] + dy) > field_lines_.y_pos_left_penalty_area) {
            continue;
          }
          error1 = std::pow(dx, 2.F) + std::pow(dy, 2.F);
          error2 = std::pow((position[0] + dx - combined_world_model_.teammate_poses.at(i).x), 2.F) +
                   std::pow((position[1] + dy - combined_world_model_.teammate_poses.at(i).y), 2.F);
          value += opt_variables_.cover_scale * std::exp(-error1 / opt_variables_.cover_variance) -
                   2.F * opt_variables_.cover_scale * std::exp(-error2 / opt_variables_.cover_variance);
        }
      }
    }

    return -value;
  }

  float TargetPoseGenerator::getBallCoMCost(const std::vector<float>& position, const std::vector<float>& des_com) const {
    float x_tot = position[0];
    float y_tot = position[1];
    int active_robots = 1;

    if (std::abs(position[0]) > field_lines_.x_pos_opponent_ground_line ||
        std::abs(position[1]) > field_lines_.y_pos_left_sideline) {
      return 1e4;
    }

    for (int i = 0; i < MAX_NUM_OF_PLAYERS - 1; i++) {
      // only do this for other active field players
      if (i != 0 && i != (player_id_ - 1) && teammates_status_.is_active.at(i)) {
        active_robots += 1;
        x_tot += combined_world_model_.teammate_poses.at(i).x;
        y_tot += combined_world_model_.teammate_poses.at(i).y;
      }
    }

    // maximum and minimum for desired CoM position are tuning parameters
    // x slightly shifted towards opposite side to be more aggressive
    const float x_com_border_min = field_lines_.x_pos_own_ground_line / 3.F;
    const float x_com_border_max = field_lines_.x_pos_opponent_ground_line / 2.F;
    const float y_com_border = field_lines_.y_pos_left_penalty_area / 2.F;

    float error =
      std::pow(
        (x_tot / static_cast<float>(active_robots) - std::min(std::max(des_com[0], x_com_border_min), x_com_border_max)),
        2.F) +
      std::pow((y_tot / static_cast<float>(active_robots) - std::min(std::max(des_com[1], -y_com_border), y_com_border)),
               2.F);

    return opt_variables_.ball_CoM_scale * error;
  }

  float TargetPoseGenerator::getAvoidanceCost(const std::vector<float>& position) const {
    float error = 0.F;
    float value = 0.F;

    for (const auto& opponent_pose : world_model_.opponent_poses) {
      error = std::pow(position[0] - opponent_pose.x, 2.F) + std::pow(position[1] - opponent_pose.y, 2.F);
      value += opt_variables_.avoidance_scale_opponent * std::exp(-error / opt_variables_.avoidance_variance_opponent);
    }

    for (auto i = 0; i < MAX_NUM_OF_PLAYERS; i++) {
      if (i == player_id_ - 1 || !teammates_status_.is_active.at(i)) {
        continue;
      }

      error = std::pow(position[0] - combined_world_model_.teammate_poses.at(i).x, 2.F) +
              std::pow(position[1] - combined_world_model_.teammate_poses.at(i).y, 2.F);
      value += opt_variables_.avoidance_scale_team * std::exp(-error / opt_variables_.avoidance_variance_team);
    }

    return value;
  }

  float TargetPoseGenerator::getGoalFreeCost(const std::vector<float>& position) const {
    Eigen::Vector2f ball_position = getBallPosition();
    if (ball_position[0] < field_lines_.center_circle_radius || position[0] < ball_position[0]) {
      return 0.F;
    }

    const float t_1 = std::atan2(-ball_position[1], field_lines_.x_pos_opponent_goal - ball_position[0]);
    const float t_2 = std::atan2(-position[1], field_lines_.x_pos_opponent_goal - position[0]);

    return -opt_variables_.goal_free_scale * std::abs(t_1 - t_2);
  }

  float TargetPoseGenerator::getOffenseDefenseCost(const std::vector<float>& position) const {
    float error = 0.F;
    float value = 0.F;
    float side = 0.F;

    for (const auto& opponent_pose : world_model_.opponent_poses) {
      side =
        (opponent_pose.x - opt_variables_.offense_defense_x_switch_behavior) / abs(field_lines_.x_pos_opponent_ground_line);

      error = std::pow(position[0] - opponent_pose.x, 2) + std::pow(position[1] - opponent_pose.y, 2);

      value += opt_variables_.offense_defense_scale * side * std::exp(-error / opt_variables_.offense_defense_variance);
    }

    return value;
  }

} // namespace nomadz_behavior
