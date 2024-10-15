#include "nomadz_behavior/action_nodes/motion_nodes/special_actions.hpp"

#include <spdlog/spdlog.h>

#include "nomadz_behavior/data_types/motion.hpp"
#include "nomadz_behavior/data_types/proprioception.hpp"
#include "nomadz_motion_control_msgs/special_action_request_enums.hpp"
#include "nomadz_behavior/data_types/ego_status.hpp"

using nomadz_motion_control_msgs::SpecialActionType;

namespace nomadz_behavior {
  BT::PortsList SpecialAction::providedPorts() {
    BT::PortsList ports_list;
    ports_list.emplace(BT::InputPort<SpecialActionType>("special_action_type"));
    return ports_list;
  }

  BT::NodeStatus SpecialAction::onStart() {
    spdlog::info("START SpecialAction.");
    // Create a Motion Request message and publish it
    BT::Expected<SpecialActionType> special_action_type = getInput<SpecialActionType>("special_action_type");
    MotionRequest motion_request = MotionRequest();
    motion_request.motion_type = MotionType::SPECIAL_ACTION;
    motion_request.special_action_request.special_action_type = special_action_type.value();
    blackboard_->set<MotionRequest>("motion_request", motion_request);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus SpecialAction::onRunning() {
    // Check if the motion is done
    spdlog::info("RUN SpecialAction.");
    if (blackboard_->get<bool>("is_motion_done")) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void SpecialAction::onHalted() {
    spdlog::info("HALTED SpecialAction.");
  }

  BT::NodeStatus Penalty::onStart() {
    spdlog::info("START Penalty.");
    // Create a Motion Request message and publish it
    MotionRequest motion_request = MotionRequest();
    motion_request.motion_type = MotionType::SPECIAL_ACTION;
    motion_request.special_action_request.special_action_type = SpecialActionType::STAND_HIGH;
    blackboard_->set<MotionRequest>("motion_request", motion_request);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus Penalty::onRunning() {
    // Check if the motion is done
    spdlog::info("RUN Penalty.");
    const auto ego_status = blackboard_->get<EgoStatus>("ego_status");
    if (ego_status.is_penalized) {
      return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::SUCCESS;
  }

  void Penalty::onHalted() {
    spdlog::info("HALTED Penalty.");
  }

  BT::NodeStatus Recovery::onStart() {
    using FallDirection = RobotPosture::FallDirection;

    spdlog::info("START Recovery.");
    // Create a Motion Request message and publish it
    auto robot_posture = blackboard_->get<RobotPosture>("robot_posture");
    MotionRequest motion_request{};
    SpecialActionRequest special_action_request{};

    // Only recover when ON GROUND
    if (robot_posture.is_on_ground) {
      switch (robot_posture.fall_direction) {
      case FallDirection::FRONT:
        special_action_request.special_action_type = SpecialActionType::GET_UP_FRONT_NAO_22;
        break;
      case FallDirection::BACK:
        if (counter_back_ > 0) {
          special_action_request.mirror = true;
          counter_back_ = 0;
        }
        special_action_request.special_action_type = SpecialActionType::GET_UP_BACK_NAO_22;
        counter_back_ += 1;
        break;
      default:
        special_action_request.special_action_type = SpecialActionType::PLAY_DEAD;
      }

      motion_request.motion_type = MotionType::SPECIAL_ACTION;
      motion_request.special_action_request = special_action_request;
      blackboard_->set<MotionRequest>("motion_request", motion_request);

      return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus Recovery::onRunning() {
    spdlog::info("RUN Recovery.");

    auto robot_posture = blackboard_->get<RobotPosture>("robot_posture");
    // Check if the motion is done
    if (blackboard_->get<bool>("is_motion_done")) {
      // check if the robot is still on the ground, if so the recovery failed
      if (robot_posture.is_on_ground) {
        MotionRequest motion_request{};
        motion_request.motion_type = MotionType::SPECIAL_ACTION;
        motion_request.special_action_request.special_action_type = SpecialActionType::PLAY_DEAD;
        blackboard_->set<MotionRequest>("motion_request", motion_request);
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void Recovery::onHalted() {
    spdlog::info("HALTED Recovery.");
  }

  BT::NodeStatus Relax::onStart() {
    spdlog::info("START RELAX.");
    MotionRequest motion_request{};
    motion_request.motion_type = MotionType::SPECIAL_ACTION;
    motion_request.special_action_request.special_action_type = SpecialActionType::PLAY_DEAD;
    blackboard_->set<MotionRequest>("motion_request", motion_request);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus Relax::onRunning() {
    spdlog::info("RUN RELAX.");
    // auto robot_posture = blackboard_->get<RobotPosture>("robot_posture");
    // auto ego_status = blackboard_->get<EgoStatus>("ego_status");
    //  Check if the motion is done

    if (blackboard_->get<bool>("is_motion_done")) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void Relax::onHalted() {
    spdlog::info("HALTED Relax.");
  }

  BT::NodeStatus SaveShot::onStart() {
    spdlog::info("START SaveShot.");
    // for now only do the sit down action
    MotionRequest motion_request{};
    motion_request.motion_type = MotionType::SPECIAL_ACTION;
    motion_request.special_action_request.special_action_type = SpecialActionType::SIT_DOWN_GOALKEEPER;
    blackboard_->set<MotionRequest>("motion_request", motion_request);

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus SaveShot::onRunning() {
    spdlog::info("RUN SaveShot.");
    if (blackboard_->get<bool>("is_motion_done")) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
  }

  void SaveShot::onHalted() {
    spdlog::info("HALTED SaveShot.");
  }

} // namespace nomadz_behavior
