#include "nomadz_behavior/action_nodes/high_level_actions.hpp"

#include <spdlog/spdlog.h>

#include "nomadz_behavior/data_types/modeling.hpp"
#include "nomadz_behavior/data_types/motion.hpp"
#include "nomadz_core/geometry/angle.hpp"

namespace nomadz_behavior {
  float getKickDirectionOffset(const nomadz_core::Pose2D robot_pose,
                               const Eigen::Vector2f& ball_pose,
                               const Eigen::Vector2f& target_position) {
    Eigen::Vector2f self_to_goal = target_position - Eigen::Vector2f{robot_pose.x, robot_pose.y};
    Eigen::Vector2f ball_to_goal = -ball_pose + self_to_goal;
    float ball_angle_to_goal = std::atan2(ball_to_goal.y(), ball_to_goal.x());

    return nomadz_core::angle::normalize(ball_angle_to_goal - robot_pose.theta);
  }

  BT::NodeStatus ApplySkill::onStart() {
    spdlog::info("START ApplySkill.");
    auto combined_world_model = blackboard_->get<CombinedWorldModel>("combined_world_model");
    auto robot_pose = blackboard_->get<WorldModel>("world_model").robot_pose;
    Eigen::Vector2f ball_pose = blackboard_->get<WorldModel>("world_model").ball_model.position;
    auto opponents_positions = combined_world_model.opponent_poses;

    bool dribble_possible = false;
    MotionRequest motion_request = MotionRequest();

    // Dribbling
    for (auto opp_pose : opponents_positions) {
      // opponent is in front of the robot
      if (opp_pose.x > robot_pose.pose.x) {
        if (nomadz_core::distance(robot_pose.pose, opp_pose) < dribble_possible_distance) {
          // an opponent is too close, dribbling is not possible
          dribble_possible = false;
          break;
        }
      }
    }

    if (dribble_possible) {
      spdlog::info("ApplySkill: Dribbling.");
      float kick_direction_offset =
        getKickDirectionOffset(robot_pose.pose, ball_pose, Eigen::Vector2f{dims_.x_pos_opponent_ground_line, 0.0});

      // generate a motion request for dribbling
      motion_request.motion_type = MotionType::WALK;
      motion_request.walk_request.mode = WalkMode::PATTERN;
      motion_request.walk_request.pattern_type = PatternType::IN_WALK_KICK;
      motion_request.kick_request.kick_power = dribble_kick_power;
      motion_request.kick_request.kick_direction = kick_direction_offset;
      blackboard_->set<MotionRequest>("motion_request", motion_request);

      return BT::NodeStatus::RUNNING;
    }

    // Shooting
    motion_request.motion_type = MotionType::WALK;
    motion_request.walk_request.mode = WalkMode::PATTERN;
    motion_request.walk_request.pattern_type = PatternType::IN_WALK_KICK;
    motion_request.kick_request.kick_power = 2.F;
    motion_request.kick_request.kick_direction =
      getKickDirectionOffset(robot_pose.pose, ball_pose, Eigen::Vector2f{dims_.x_pos_opponent_ground_line, 0.0});
    blackboard_->set<MotionRequest>("motion_request", motion_request);

    // spdlog::info("ApplySkill: Shooting.");
    // motion_request.motion_type = MotionType::KICK;
    // motion_request.kick_request.kick_type = KickType::DEFAULT;
    // motion_request.kick_request.kick_power =
    //   1.0; // probably better to always shoot hard rather than making it dependent on distance to goal
    // motion_request.kick_request.kick_direction = 0.0;
    // blackboard_->set<MotionRequest>("motion_request", motion_request);

    return BT::NodeStatus::RUNNING;

    // pass not implemented but potential ideas below
    //  check the position of the closest opponents, if they are far enough away, dribble
    //  if the closest opponent is far enough away dribble
    //  passing: weighting based on:
    //  1. distance from team member to self (sweetspot around 1m with falloff to both directions)
    //  2. positioning of team mate in relation to goal (the more central and the close the better)
    //  3. position of opponents in relation to team mate (the further away the better)
    //  in general some more bounds should be place on back passes (don't consider teammates in own half)
  }

  BT::NodeStatus ApplySkill::onRunning() {
    spdlog::info("RUNNING ApplySkill.");

    if (blackboard_->get<bool>("is_motion_done")) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
  }

  void ApplySkill::onHalted() {
    spdlog::info("HALTED ApplySkill.");
  }

  BT::NodeStatus Engage::tick() {
    spdlog::info("TICK Engage.");

    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus Kick::onStart() {
    spdlog::info("START Kick.");
    Eigen::Vector2f ball_position = blackboard_->get<WorldModel>("world_model").ball_model.position;
    nomadz_core::Pose2D robot_pose = blackboard_->get<WorldModel>("world_model").robot_pose.pose;

    MotionRequest motion_request = MotionRequest();

    // in walk kick
    motion_request.motion_type = MotionType::WALK;
    motion_request.walk_request.mode = WalkMode::PATTERN;
    motion_request.walk_request.pattern_type = PatternType::IN_WALK_KICK;
    motion_request.kick_request.kick_power = penalty_kick_power_;
    motion_request.kick_request.kick_direction =
      getKickDirectionOffset(robot_pose, ball_position, Eigen::Vector2f{dims_.x_pos_opponent_ground_line, 0.0});
    blackboard_->set<MotionRequest>("motion_request", motion_request);

    // normal kick
    // motion_request.motion_type = MotionType::KICK;
    // motion_request.kick_request.kick_type = KickRequest::KickType::IN;
    // motion_request.kick_request.kick_power =
    //   1.0; // probably better to always shoot hard rather than making it dependent on distance to goal
    // motion_request.kick_request.kick_direction = 0.0;
    // blackboard_->set<MotionRequest>("motion_request", motion_request);

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus Kick::onRunning() {
    spdlog::info("RUNNING Kick.");

    if (blackboard_->get<bool>("is_motion_done")) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
  }

  void Kick::onHalted() {
    spdlog::info("HALTED Kick.");
  }
} // namespace nomadz_behavior
