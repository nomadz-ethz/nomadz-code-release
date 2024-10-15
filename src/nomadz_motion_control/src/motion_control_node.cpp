#include "nomadz_motion_control/motion_control_node.hpp"

#include <chrono>
#include <memory>
#include <tf2_eigen/tf2_eigen.hpp>

#include "nomadz_motion_control/special_action_generator.hpp"
#include "nomadz_motion_control/kick_engine/kick_engine.hpp"
#include "nomadz_motion_control/arm_motion_engine.hpp"
#include "nomadz_motion_control/head_engine.hpp"
#include "nomadz_motion_control/walk_engine/walk_engine.hpp"
#include "nomadz_motion_control_msgs/motion_request_enums.hpp"
#include "nomadz_core/math/constants.hpp"
#include "nomadz_kinematics/ros_conversion.hpp"
#include "nomadz_motion_control_msgs/msg/walk_request.hpp"

#include "nomadz_proprioception_msgs/fall_down_state_enums.hpp"

using nomadz_motion_control_msgs::MotionType;
using nomadz_motion_control_msgs::SpecialActionType;
using nomadz_motion_control_msgs::WalkMode;

namespace nomadz_motion_control {
  MotionControlNode::MotionControlNode(const rclcpp::NodeOptions& options) : MotionControlNode("motion_node", options){};
  MotionControlNode::MotionControlNode(const std::string& node_name, const rclcpp::NodeOptions& options)
      : Node(node_name, options), active_controller_(MotionType::SPECIAL_ACTION),
        previous_controller_(MotionType::SPECIAL_ACTION) {
    loadController(options);

    // NOTE(@naefjo): Initialize the motion request in the initially active controller so there is no undefined behavior.
    MotionRequestMsgT initial_motion_request;
    initial_motion_request.special_action_request.special_action_type = static_cast<uint8_t>(SpecialActionType::STAND_HIGH);
    current_motion_request_msg_ = initial_motion_request;
    cached_motion_request_ = initial_motion_request;

    joint_requests_pub_ = this->create_publisher<JointRequestMsgT>("effectors/joint_requests", 5);
    motion_info_pub_ = this->create_publisher<MotionInfoMsgT>("motion_control/motion_info", 5);
    requested_robot_model_pub_ = this->create_publisher<RobotModelMsgT>("motion_control/requested_robot_model", 5);

    motion_request_sub_ = this->create_subscription<MotionRequestMsgT>(
      "behavior/motion_request", 1, [this](MotionRequestMsgT::ConstSharedPtr msg) {
        cached_motion_request_ = *msg;
        current_motion_request_msg_.head_motion_request = cached_motion_request_.head_motion_request;
        if (static_cast<MotionType>(msg->motion_type) != previous_controller_) {
          motion_controller_map_.at(active_controller_)->requestLeave();
          RCLCPP_INFO(this->get_logger(), "Waiting for controller: %u to finish", active_controller_);

          requested_controller_switch_ = true;
        }
      });

    fall_down_state_sub_ = this->create_subscription<FallDownStateMsgT>(
      "proprioception/fall_down_state", 1, [this](FallDownStateMsgT::ConstSharedPtr msg) {
        processFalldownState(msg);
        motionControlCallback();
      });

    unstiff_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "sensors/unstiff_state", 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        unstiffed_ = msg->data;
        if (msg->data) {
          motion_controller_map_.at(active_controller_)->reset();
        }
      });
  }

  void MotionControlNode::overrideMotionRequest(const MotionRequestMsgT& motion_request_override) {
    motion_controller_map_.at(active_controller_)->reset();
    current_motion_request_msg_ = motion_request_override;
    previous_controller_ = static_cast<MotionType>(motion_request_override.motion_type);
    auto new_controller = static_cast<MotionType>(current_motion_request_msg_.motion_type);
    motion_controller_map_.at(new_controller)->reset();
    active_controller_ = new_controller;
    has_motion_request_been_overwritten_ = true;
    motionControlCallback();
  }

  void MotionControlNode::loadController(const rclcpp::NodeOptions& options) {
    motion_controller_map_.insert({MotionType::SPECIAL_ACTION, std::make_shared<SpecialActionGenerator>(options)});
    motion_controller_map_.insert({MotionType::WALK, std::make_shared<WalkEngine>(options)});
    motion_controller_map_.insert({MotionType::KICK, std::make_shared<KickEngine>(options)});

    arm_motion_engine_ = std::make_shared<ArmMotionEngine>(options);
    head_motion_engine_ = std::make_shared<HeadEngine>(options);
  }

  void MotionControlNode::updateController() {
    auto current_controller_ptr = motion_controller_map_.at(active_controller_);
    auto new_controller = static_cast<MotionType>(cached_motion_request_.motion_type);

    // If we can leave the current controller, then we will switch to the requested one
    if (current_controller_ptr->isLeavingPossible()) {
      requested_controller_switch_ = false;
      has_motion_request_been_overwritten_ = false;
      current_motion_request_msg_ = cached_motion_request_;
      motion_controller_map_.at(new_controller)->reset();
      active_controller_ = new_controller;

      RCLCPP_INFO(this->get_logger(), "Switch to motiontype: %u", new_controller);
    }
  }

  void MotionControlNode::processFalldownState(FallDownStateMsgT::ConstSharedPtr msg) {
    using nomadz_proprioception_msgs::FallDirection;
    using nomadz_proprioception_msgs::FallDownState;

    is_on_ground_ = msg->fall_down_state == static_cast<uint8_t>(FallDownState::ON_GROUND);
    if (msg->fall_down_state == static_cast<uint8_t>(FallDownState::UNDEFINED) ||
        msg->fall_down_state == static_cast<uint8_t>(FallDownState::UPRIGHT)) {
      return;
    }

    auto current_controller = motion_controller_map_.at(active_controller_);

    bool is_falling = msg->fall_down_state == static_cast<uint8_t>(FallDownState::FALLING);
    bool is_in_special_action = active_controller_ == MotionType::SPECIAL_ACTION;
    bool is_current_motion_interruptable = current_controller->isLeavingPossible();

    // NOTE: If we are in a non interruptible special action, we do nothing.
    if (!is_falling || (is_in_special_action && !is_current_motion_interruptable)) {
      return;
    }

    active_controller_ = MotionType::SPECIAL_ACTION;

    MotionRequestMsgT falldown_protection_motion_request;
    falldown_protection_motion_request.motion_type = static_cast<uint8_t>(MotionType::SPECIAL_ACTION);
    falldown_protection_motion_request.special_action_request.mirror = false;
    SpecialActionType special_action_type = SpecialActionType::PLAY_DEAD;
    switch (static_cast<FallDirection>(msg->fall_direction)) {
    case FallDirection::BACK:
      special_action_type = SpecialActionType::FALL_PROTECTION_BACK;
      break;
    case FallDirection::FRONT:
      special_action_type = SpecialActionType::FALL_PROTECTION_FRONT;
      break;
    case FallDirection::LEFT:
    case FallDirection::RIGHT:
      special_action_type = SpecialActionType::FALL_PROTECTION_SIDE;
      break;
    default:
      break;
      RCLCPP_WARN(this->get_logger(), "Falldown was detected but no direction was provided.");
    }
    falldown_protection_motion_request.special_action_request.special_action_type =
      static_cast<uint8_t>(special_action_type);
    overrideMotionRequest(falldown_protection_motion_request);
  }

  void MotionControlNode::motionControlCallback() {
    if (requested_controller_switch_ || has_motion_request_been_overwritten_) {
      updateController();
    } else {
      current_motion_request_msg_ = cached_motion_request_;
    }

    std::shared_ptr<MotionBase> currently_active_controller = motion_controller_map_.at(active_controller_);

    executor_.spin_node_some(currently_active_controller);
    executor_.spin_node_some(arm_motion_engine_);
    executor_.spin_node_some(head_motion_engine_);

    currently_active_controller->executeControlLoop(current_motion_request_msg_);
    arm_motion_engine_->executeControlLoop(current_motion_request_msg_);
    head_motion_engine_->executeControlLoop(current_motion_request_msg_);

    JointRequests active_ctrl_jr = currently_active_controller->getJointRequest();
    JointRequests arm_engine_jr = arm_motion_engine_->getJointRequest();
    JointRequests head_engine_jr = head_motion_engine_->getJointRequest();

    JointRequests joint_requests = JointRequests::prioritizedCombination(active_ctrl_jr, arm_engine_jr);
    joint_requests = JointRequests::prioritizedCombination(joint_requests, head_engine_jr);

    interpolator_.process(joint_requests, active_controller_ != previous_controller_);

    joint_requests_pub_->publish(joint_requests.toJointRequestsMsg(this->now()));

    MotionInfoMsgT motion_info = currently_active_controller->getMotionInfo();
    motion_info.header.stamp = this->now();
    motion_info.is_head_motion_done = head_motion_engine_->getMotionInfo().is_head_motion_done;
    motion_info.executed_motion_request.head_motion_request =
      head_motion_engine_->getMotionInfo().executed_motion_request.head_motion_request;
    motion_info_pub_->publish(motion_info);

    requested_robot_model_.setJointPositions(joint_requests.positions.values);
    requested_robot_model_pub_->publish(nomadz_kinematics::packRobotModel(requested_robot_model_, this->now()));

    previous_controller_ = active_controller_;
  }

} // namespace nomadz_motion_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nomadz_motion_control::MotionControlNode)
