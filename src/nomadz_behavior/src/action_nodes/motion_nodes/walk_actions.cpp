#include "nomadz_behavior/action_nodes/motion_nodes/walk_actions.hpp"

#include <spdlog/spdlog.h>

#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_core/geometry/twist.hpp"
#include "nomadz_configuration/game_settings.hpp"
#include "nomadz_behavior/helper/bt_conversion.hpp"
#include "nomadz_behavior/helper/path_planner.hpp"
#include "nomadz_behavior/data_types/modeling.hpp"
#include "nomadz_behavior/helper/common_functions.hpp"

using Pose2D = nomadz_core::Pose2D;
using Twist2D = nomadz_core::Twist2D;

namespace nomadz_behavior {
  BT::PortsList WalkAtSpeed::providedPorts() {
    BT::PortsList ports_list;
    ports_list.emplace(BT::InputPort<Twist2D>("target_speed"));
    return ports_list;
  }

  BT::NodeStatus WalkAtSpeed::tick() {
    spdlog::info("TICK WalkAtSpeed.");
    BT::Expected<Twist2D> target_speed = getInput<Twist2D>("target_speed");

    MotionRequest motion_request{};
    motion_request.motion_type = MotionType::WALK;
    motion_request.walk_request.mode = WalkMode::SPEED;
    motion_request.walk_request.target_speed = target_speed.value();
    blackboard_->set<MotionRequest>("motion_request", motion_request);
    return BT::NodeStatus::SUCCESS;
  }

  BT::PortsList WalkToTarget::providedPorts() {
    BT::PortsList ports_list;

    ports_list.emplace(BT::InputPort<Pose2D>("target"));

    return ports_list;
  }

  BT::NodeStatus WalkToTarget::onStart() {
    spdlog::info("START WalkToTarget.");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus WalkToTarget::onRunning() {
    spdlog::info("RUN WalkToTarget.");
    BT::Expected<Pose2D> target_pose = getInput<Pose2D>("target");
    Pose2D current_pose = blackboard_->get<WorldModel>("world_model").robot_pose.pose;
    Twist2D target_speed = calculateSpeedFromTargetPose(current_pose, target_pose.value());

    MotionRequest motion_request = MotionRequest();
    motion_request.motion_type = MotionType::WALK;
    motion_request.walk_request.mode = WalkMode::SPEED;
    motion_request.walk_request.target_speed = target_speed;
    blackboard_->set<MotionRequest>("motion_request", motion_request);

    Twist2D pose_diff = target_pose.value() - current_pose;
    const float distance = std::sqrt(std::pow(pose_diff.x, 2) + std::pow(pose_diff.y, 2));
    if (distance < 0.3F && std::abs(pose_diff.theta) < 0.1F) {
      motion_request.walk_request.target_speed = Twist2D{};
      blackboard_->set<MotionRequest>("motion_request", motion_request);
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::RUNNING;
    }
  }

  void WalkToTarget::onHalted() {
    spdlog::info("HALTED WalkToTarget.");
  }

  BT::PortsList Alignment::providedPorts() {
    BT::PortsList ports_list;
    ports_list.emplace(BT::InputPort<std::string>("set_play"));

    return ports_list;
  };

  BT::NodeStatus Alignment::onStart() {
    spdlog::info("START Alignment.");
    set_play_ = getInput<std::string>("set_play").value();
    const auto motion_tuple = calculateMotionRequest(set_play_);
    MotionRequest motion_request = std::get<0>(motion_tuple);
    blackboard_->set<MotionRequest>("motion_request", motion_request);
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus Alignment::onRunning() {
    spdlog::info("RUN Alignment.");
    auto ball_pose = blackboard_->get<WorldModel>("world_model").ball_model;
    Pose2D robot_pose = blackboard_->get<WorldModel>("world_model").robot_pose.pose;
    if (ball_pose.position.norm() >= 0.5) {
      return BT::NodeStatus::FAILURE;
    }
    const auto motion_tuple = calculateMotionRequest(set_play_);
    if (std::fabs(std::get<1>(motion_tuple) - std::get<2>(motion_tuple)) < angular_tolerance &&
        std::fabs(nomadz_core::angle::normalize(robot_pose.theta) - std::get<1>(motion_tuple)) < angular_tolerance) {
      return BT::NodeStatus::SUCCESS;
    }

    MotionRequest motion_request = std::get<0>(motion_tuple);
    blackboard_->set<MotionRequest>("motion_request", motion_request);
    return BT::NodeStatus::RUNNING;
  }

  std::tuple<MotionRequest, float, float> Alignment::calculateMotionRequest(std::string set_play) {
    Pose2D robot_pose = blackboard_->get<WorldModel>("world_model").robot_pose.pose;
    auto ball_pose = blackboard_->get<WorldModel>("world_model").ball_model.position;

    Eigen::Vector2f ball_to_goal;
    if (set_play == "corner_kick") {
      ball_to_goal = opp_penalty_mark_ - nomadz_behavior::globalFromLocal(ball_pose, robot_pose);
    } else {
      ball_to_goal = opp_goal_center_ - nomadz_behavior::globalFromLocal(ball_pose, robot_pose);
    }
    float angle_ball_goal = std::atan2(ball_to_goal.y(), ball_to_goal.x());
    float angle_diff = nomadz_core::angle::normalize(angle_ball_goal - robot_pose.theta);
    float angle_robot_ball = std::atan2(ball_pose[1] - robot_pose.y, ball_pose[0] - robot_pose.x);
    float y_diff = ball_pose.norm() * static_cast<float>(std::sin(angle_diff * 0.5)); // stretch sin curve from 0 to 180
    auto x_diff = static_cast<float>(std::sin(0.5 * (ball_pose.x() - 0.18F)));
    MotionRequest motion_request = MotionRequest();
    motion_request.motion_type = MotionType::WALK;
    motion_request.walk_request.mode = WalkMode::SPEED;
    motion_request.walk_request.target_speed =
      Twist2D{1.5F * x_diff, -(y_diff + std::copysign(0.1F, y_diff)), 1.5F * angle_diff};
    return std::tuple<MotionRequest, float, float>{motion_request, angle_ball_goal, angle_robot_ball};
  }

  void Alignment::onHalted() {
    spdlog::info("HALTED Alignment.");
  }

  BT::NodeStatus FootAlignment::onStart() {
    spdlog::info("START FootAlignment.");
    MotionRequest motion_request = MotionRequest();
    motion_request.motion_type = MotionType::WALK;
    motion_request.walk_request.mode = WalkMode::PATTERN;
    motion_request.walk_request.pattern_type = PatternType::ALIGNMENT;
    motion_request.walk_request.target_speed = Twist2D{}; // TODO (@frede791) Check if a target speed is needed
    blackboard_->set<MotionRequest>("motion_request", motion_request);

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus FootAlignment::onRunning() {
    spdlog::info("RUN FootAlignment.");
    if (blackboard_->get<bool>("is_motion_done")) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
  }

  void FootAlignment::onHalted() {
    spdlog::info("HALTED FootAlignment.");
  }

} // namespace nomadz_behavior
