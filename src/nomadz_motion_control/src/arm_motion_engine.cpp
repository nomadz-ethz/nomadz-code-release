#include "nomadz_motion_control/arm_motion_engine.hpp"

#include "nomadz_motion_control_msgs/motion_request_enums.hpp"

namespace nomadz_motion_control {
  ArmMotionEngine::ArmMotionEngine(const rclcpp::NodeOptions& options)
      : MotionBase("arm_motion_engine", options), param_listener_(get_node_parameters_interface()), contact_delay_(0, 0) {
    initParameters();
    // Initialize joint ignores
    for (auto& joint : current_joint_requests_.joint_ignore) {
      joint = true;
    }
    for (int i = 0; i < 6; i++) {
      current_joint_requests_.joint_ignore[kleft_idxs_[i]] = false;
      current_joint_requests_.joint_ignore[kright_idxs_[i]] = false;
    }

    foot_poses_ = {Eigen::Array2f::Zero(2), Eigen::Array2f::Zero(2)};
    t_last_contact_ = {rclcpp::Time(), rclcpp::Time()};

    requested_robot_model_sub_ = this->create_subscription<RobotModelMsgT>(
      "motion_control/requested_robot_model", 1, [this](RobotModelMsgT::ConstSharedPtr msg) {
        foot_poses_.left[0] = static_cast<float>(msg->foot_poses[0].position.x);
        foot_poses_.left[1] = static_cast<float>(msg->foot_poses[0].position.y);
        foot_poses_.right[0] = static_cast<float>(msg->foot_poses[1].position.x);
        foot_poses_.right[1] = static_cast<float>(msg->foot_poses[1].position.y);
      });

    arm_contact_sub_ = this->create_subscription<ArmContactModelMsgT>(
      "proprioception/arm_contact", 1, [this](ArmContactModelMsgT::ConstSharedPtr msg) {
        t_last_contact_.left = (msg->contact_left) ? rclcpp::Clock().now() : t_last_contact_.left;
        t_last_contact_.right = (msg->contact_right) ? rclcpp::Clock().now() : t_last_contact_.right;
      });
  }

  void ArmMotionEngine::updateJointRequests() {
    updateEndpoints();
    const ArmPairT<MotionMode> motion_modes = computeDesiredMode();
    const ArmPairT<EigenArr6f> setpoints = computeSetpoints(motion_modes);
    setJointPoints(setpoints);
  }

  void ArmMotionEngine::updateMotionInfoMsg() {}

  bool ArmMotionEngine::isLeavingPossible() {
    return true;
  }

  void ArmMotionEngine::reset() {}

  // ******** AME Member Functions ********

  void ArmMotionEngine::updateEndpoints() {
    // Adjust shoulder roll acc. to the robot's heading and the pitch acc. to the foot positions.
    auto heading = static_cast<float>(std::atan2(std::abs(current_motion_request_msg_.walk_request.target_speed.linear.y),
                                                 std::abs(current_motion_request_msg_.walk_request.target_speed.linear.x)));
    float shoulder_roll_offset = shoulder_roll_offset_ * heading / static_cast<float>(M_PI_2);

    EigenArr6f swing_endpoint = mode_endpoints_.left[MotionMode::K_SWING];
    swing_endpoint(0) = mode_endpoints_.left[MotionMode::K_DEFAULT](0) + shoulder_roll_offset;
    swing_endpoint(1) = M_PI_2 - swing_gain_ * foot_poses_.right.x();
    mode_endpoints_.left[MotionMode::K_SWING] = swing_endpoint;

    swing_endpoint = mode_endpoints_.right[MotionMode::K_SWING];
    swing_endpoint(0) = mode_endpoints_.right[MotionMode::K_DEFAULT](0) - shoulder_roll_offset;
    swing_endpoint(1) = M_PI_2 - swing_gain_ * foot_poses_.left.x();
    mode_endpoints_.right[MotionMode::K_SWING] = swing_endpoint;
  }

  ArmPairT<MotionMode> ArmMotionEngine::computeDesiredMode() {
    ArmPairT<MotionMode> motion_modes = {MotionMode::K_DEFAULT, MotionMode::K_DEFAULT};
    bool is_walking = static_cast<MotionType>(current_motion_request_msg_.motion_type) == MotionType::WALK;
    if (is_walking) {
      motion_modes.left = MotionMode::K_SWING;
      motion_modes.right = MotionMode::K_SWING;
    }
    if ((rclcpp::Clock().now() - t_last_contact_.left) < contact_delay_) {
      motion_modes.left = MotionMode::K_TO_BACK;
    }
    if ((rclcpp::Clock().now() - t_last_contact_.right) < contact_delay_) {
      motion_modes.right = MotionMode::K_TO_BACK;
    }
    return motion_modes;
  }

  ArmPairT<EigenArr6f> ArmMotionEngine::computeSetpoints(const ArmPairT<MotionMode> motion_modes) {
    ArmPairT<EigenArr6f> current_point = getJointPoints();
    ArmPairT<EigenArr6f> setpoints = {
      propagatePoint(motion_modes.left, current_point.left, mode_endpoints_.left[motion_modes.left]),
      propagatePoint(motion_modes.right, current_point.right, mode_endpoints_.right[motion_modes.right])};
    return setpoints;
  }

  EigenArr6f
  ArmMotionEngine::propagatePoint(const MotionMode motion_mode, const EigenArr6f current_point, const EigenArr6f endpoint) {
    EigenArr6f error = endpoint - current_point;
    EigenArr6f u = p_gains_ * error;

    EigenArr6f setpoint = current_point + u;

    // Set roll and pitch priority to not collide with the robot body.
    if (motion_mode == MotionMode::K_TO_BACK && (std::abs(error[1]) > thrsh_shoulder_pitch_)) {
      setpoint[0] = current_point[0];
    } else if (!(motion_mode == MotionMode::K_TO_BACK) && (std::abs(error[0]) > thrsh_shoulder_roll_)) {
      setpoint[1] = current_point[1];
    }
    return setpoint;
  }

  ArmPairT<EigenArr6f> ArmMotionEngine::getJointPoints() {
    EigenArr6f left_joint_point = EigenArr6f::Zero(6);
    EigenArr6f right_joint_point = EigenArr6f::Zero(6);

    for (int i = 0; i < 6; i++) {
      left_joint_point(i) = current_joint_data_.positions.values[kleft_idxs_[i]];
      right_joint_point(i) = current_joint_data_.positions.values[kright_idxs_[i]];
    }
    ArmPairT<EigenArr6f> joint_points = {left_joint_point, right_joint_point};
    return joint_points;
  }

  void ArmMotionEngine::setJointPoints(const ArmPairT<EigenArr6f> joint_points) {
    for (int i = 0; i < 6; i++) {
      current_joint_requests_.positions.values[kleft_idxs_[i]] = joint_points.left(i);
      current_joint_requests_.positions.values[kright_idxs_[i]] = joint_points.right(i);
    }
  }

  EigenArr6f ArmMotionEngine::mirrorPoint(const EigenArr6f joint_point) {
    EigenArr6f new_point = joint_point;
    new_point(0) = -new_point(0);
    new_point(2) = -new_point(2);
    new_point(3) = -new_point(3);
    new_point(4) = -new_point(4);
    return new_point;
  }

  void ArmMotionEngine::initParameters() {
    auto params = param_listener_.get_params();

    swing_gain_ = static_cast<float>(params.swing_gain);
    thrsh_shoulder_roll_ = static_cast<float>(params.thrsh_shoulder_roll);
    thrsh_shoulder_pitch_ = static_cast<float>(params.thrsh_shoulder_pitch);
    shoulder_roll_offset_ = static_cast<float>(params.shoulder_roll_offset);

    auto contact_delay_param = std::chrono::milliseconds(params.contact_delay_ms);
    contact_delay_ = rclcpp::Duration(contact_delay_param);

    EigenArr6f left_default_point;
    EigenArr6f left_back_point;
    for (int i = 0; i < 6; i++) {
      p_gains_(i) = static_cast<float>(params.p_gains[i]);
      left_default_point(i) = static_cast<float>(params.left_default_point[i]);
      left_back_point(i) = static_cast<float>(params.left_back_point[i]);
      current_joint_requests_.stiffnesses.values[kleft_idxs_[i]] = static_cast<float>(params.stiffness[i]);
      current_joint_requests_.stiffnesses.values[kright_idxs_[i]] = static_cast<float>(params.stiffness[i]);
    }

    std::map<MotionMode, EigenArr6f> left_mode_endpoints = {{MotionMode::K_SWING, left_default_point},
                                                            {MotionMode::K_TO_BACK, left_back_point},
                                                            {MotionMode::K_DEFAULT, left_default_point}};
    std::map<MotionMode, EigenArr6f> right_mode_endpoints = {{MotionMode::K_SWING, mirrorPoint(left_default_point)},
                                                             {MotionMode::K_TO_BACK, mirrorPoint(left_back_point)},
                                                             {MotionMode::K_DEFAULT, mirrorPoint(left_default_point)}};
    mode_endpoints_ = {left_mode_endpoints, right_mode_endpoints};
  }

} // namespace nomadz_motion_control
