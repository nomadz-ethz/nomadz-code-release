#include "nomadz_behavior/check_nodes/check_nodes.hpp"

#include <spdlog/spdlog.h>

#include "nomadz_behavior/data_types/ego_status.hpp"
#include "nomadz_behavior/data_types/game_status.hpp"
#include "nomadz_behavior/data_types/modeling.hpp"
#include "nomadz_behavior/data_types/proprioception.hpp"
#include "nomadz_behavior/helper/common_functions.hpp"
#include "nomadz_core/geometry/angle.hpp"

using SetPlay = nomadz_communication_msgs::SetPlay;

namespace nomadz_behavior {
  BT::NodeStatus Penalized::tick() {
    spdlog::info("TICK Penalized.");
    auto ego_status = blackboard_->get<EgoStatus>("ego_status");
    if (ego_status.is_penalized) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus Standing::tick() {
    spdlog::info("TICK Standing.");
    auto robot_posture = blackboard_->get<RobotPosture>("robot_posture");
    if (robot_posture.is_standing) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus Localized::tick() {
    spdlog::info("TICK Localized.");
    auto robot_pose = blackboard_->get<WorldModel>("world_model").robot_pose;
    if (robot_pose.valid) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus Lost::tick() {
    spdlog::info("TICK Lost.");
    auto robot_pose = blackboard_->get<WorldModel>("world_model").robot_pose;
    if (robot_pose.lost) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus BallFound::tick() {
    spdlog::info("TICK BallFound.");
    auto ball_model = blackboard_->get<WorldModel>("world_model").ball_model;
    if (ball_model.valid) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus BallLost::tick() {
    spdlog::info("TICK BallLost.");
    auto ball_model = blackboard_->get<WorldModel>("world_model").ball_model;
    if (ball_model.lost) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus HasBallLock::tick() {
    spdlog::info("TICK HasBallLock.");
    auto ego_status = blackboard_->get<EgoStatus>("ego_status");
    if (ego_status.has_ball_lock) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus BallFree::tick() {
    spdlog::info("TICK BallFree.");
    SetPlay set_play = blackboard_->get<GameStatus>("game_status").set_play;
    bool kicking_team = blackboard_->get<GameStatus>("game_status").is_kicking_team;

    if (set_play == SetPlay::NONE || (set_play != SetPlay::NONE && kicking_team)) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus HasBall::tick() {
    spdlog::info("TICK HasBall.");
    auto ball_distance = blackboard_->get<WorldModel>("world_model").ball_model.position.norm();

    if (ball_distance <= has_ball_distance) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus IsHighestActivePlayer::tick() {
    spdlog::info("TICK IsHighestActivePlayer.");

    if (getNumberHigherActiveFieldPlayers(player_id_, blackboard_->get<TeammatesStatus>("teammates_status").is_active) > 0) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus IsAligned::tick() {
    spdlog::info("TICK IsAligned.");
    const nomadz_core::Pose2D robot_pose = blackboard_->get<WorldModel>("world_model").robot_pose.pose;
    const Eigen::Vector2f ball_position =
      globalFromLocal(blackboard_->get<WorldModel>("world_model").ball_model.position, robot_pose);
    const float angle_ball_goal = std::atan2(0.F - ball_position[1], field_lines.x_pos_opponent_goal - ball_position[0]);
    const float angle_robot_ball = std::atan2(ball_position[1] - robot_pose.y, ball_position[0] - robot_pose.x);
    if (std::fabs(angle_ball_goal - angle_robot_ball) < angular_tolerance &&
        std::fabs(nomadz_core::angle::normalize(robot_pose.theta) - angle_ball_goal) < angular_tolerance) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus CanSaveShot::tick() {
    spdlog::info("TICK CanSaveShot.");
    const auto ball_model = blackboard_->get<WorldModel>("world_model").ball_model;
    const auto field_lines = blackboard_->get<nomadz_configuration::FieldLines>("field_lines");

    const float distance_keeper_ball = ball_model.position.norm();

    if (distance_keeper_ball <= distance_keeper_penalty_mark * 0.9F && ball_model.velocity[0] < 0.F && ball_model.valid) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus IsInsideArea::tick() {
    spdlog::info("TICK IsInsideArea.");
    auto robot_pose = blackboard_->get<WorldModel>("world_model").robot_pose;

    bool y_out_of_bounds =
      robot_pose.pose.y > left_penalty_area_corner.y() || robot_pose.pose.y < right_penalty_area_corner.y();
    bool x_out_of_bounds = robot_pose.pose.x > left_penalty_area_corner.x();

    if (y_out_of_bounds || x_out_of_bounds) {
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }
} // namespace nomadz_behavior
