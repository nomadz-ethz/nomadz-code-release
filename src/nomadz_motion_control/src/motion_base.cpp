#include "nomadz_motion_control/motion_base.hpp"

#include <chrono>

namespace nomadz_motion_control {

  MotionBase::MotionBase(const std::string& node_name, const rclcpp::NodeOptions& options) : Node(node_name, options) {
    joint_data_sub_ = this->create_subscription<JointDataMsgT>(
      "sensors/joint_data", 1, [this](JointDataMsgT::ConstSharedPtr msg) { current_joint_data_ = JointData(*msg); });

    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
    parameter_descriptor.description = "Whether the testing subscription to the motion_request should be loaded";
    parameter_descriptor.read_only = true;
    this->declare_parameter("debug_motion_request_sub", false, parameter_descriptor);
    const bool debug_mode = this->get_parameter("debug_motion_request_sub").as_bool();

    if (debug_mode) {
      motion_request_sub_ = this->create_subscription<MotionRequestMsgT>(
        "behavior/motion_request", 1, [this](MotionRequestMsgT::ConstSharedPtr msg) {
          RCLCPP_WARN_ONCE(this->get_logger(),
                           "The MotionBase motion_request subscriber should only be used for testing purposes.");
          current_motion_request_msg_ = *msg;
        });
    }
  }

  JointRequests MotionBase::getJointRequest() const {
    return current_joint_requests_;
  }

  MotionBase::MotionInfoMsgT MotionBase::getMotionInfo() const {
    return motion_info_msg_;
  }

  void MotionBase::executeControlLoop() {
    updateJointRequests();
    updateMotionInfoMsg();
    JointRequests::updateJointRequestWithMask(current_joint_data_);
    last_joint_requests_ = current_joint_requests_;
  }

  void MotionBase::executeControlLoop(const MotionRequestMsgT& motion_request) {
    current_motion_request_msg_ = motion_request;
    executeControlLoop();
  }

} // namespace nomadz_motion_control
