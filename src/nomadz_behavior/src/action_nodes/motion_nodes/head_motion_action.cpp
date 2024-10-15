#include "nomadz_behavior/action_nodes/motion_nodes/head_motion_action.hpp"

#include <spdlog/spdlog.h>

#include "nomadz_behavior/data_types/motion.hpp"
#include "nomadz_behavior/data_types/modeling.hpp"

using Eigen::Vector2f;
using Eigen::Vector3f;

namespace nomadz_behavior {
  BT::NodeStatus LookAtBall::onStart() {
    spdlog::info("START LookAtBall.");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus LookAtBall::onRunning() {
    BallModel ball_model = blackboard_->get<WorldModel>("world_model").ball_model;

    HeadMotionRequest head_motion_request{};
    head_motion_request.head_motion_type = HeadMotionType::TARGET;

    head_motion_request.target = Vector3f{ball_model.position.x(),
                                          ball_model.position.y(),
                                          -0.26F}; // TODO(@ilgonhciz @frede791 use the actual z from the camera transform)
    blackboard_->set<HeadMotionRequest>("head_motion_request", head_motion_request);

    if (ball_model.lost) {
      return BT::NodeStatus::FAILURE;
    } else {
      return BT::NodeStatus::RUNNING;
    }
  }

  void LookAtBall::onHalted() {
    spdlog::info("HALTED LookAtBall.");
  }

  BT::PortsList LookAtDirection::providedPorts() {
    BT::PortsList ports_list;
    ports_list.emplace(BT::InputPort<float>("look_target_direction"));
    return ports_list;
  }

  BT::NodeStatus LookAtDirection::onStart() {
    spdlog::info("TICK LookAtDirection.");
    float look_target_direction = getInput<float>("look_target_direction").value();
    HeadMotionRequest head_motion_request{};
    head_motion_request.head_motion_type = HeadMotionType::DIRECT;
    head_motion_request.pan = look_target_direction;
    head_motion_request.tilt = 0.2F;
    blackboard_->set<HeadMotionRequest>("head_motion_request", head_motion_request);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus LookAtDirection::onRunning() {
    if (blackboard_->get<bool>("is_head_motion_done")) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void LookAtDirection::onHalted() {
    spdlog::info("HALTED LookAtDirection.");
  }
} // namespace nomadz_behavior
