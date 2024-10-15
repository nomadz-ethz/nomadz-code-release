#include "nomadz_motion_control/head_engine.hpp"

#include "nomadz_motion_control_msgs/head_motion_request_enums.hpp"

using nomadz_motion_control_msgs::HeadMotionType;

namespace nomadz_motion_control {
  HeadEngine::HeadEngine(const rclcpp::NodeOptions& options)
      : MotionBase("head_engine", options), param_listener_(get_node_parameters_interface()) {
    initParameters();
    // Initialize joint ignores
    for (auto& joint : current_joint_requests_.joint_ignore) {
      joint = true;
    }
    current_joint_requests_.joint_ignore[nomadz_definitions::joint_indexes::HEAD_PITCH] = false;
    current_joint_requests_.joint_ignore[nomadz_definitions::joint_indexes::HEAD_YAW] = false;
  }

  void HeadEngine::updateJointRequests() {

    // Limit the angles to [-pi, pi]
    float current_pan = current_joint_data_.positions.values[nomadz_definitions::joint_indexes::HEAD_YAW];
    float current_tilt = current_joint_data_.positions.values[nomadz_definitions::joint_indexes::HEAD_PITCH];
    current_pan -= 2 * M_PI * std::floor((current_pan + M_PI) * (1. / (2 * M_PI)));
    current_tilt -= 2 * M_PI * std::floor((current_tilt + M_PI) * (1. / (2 * M_PI)));

    // Update tilt bound based on current pan
    float projected_tilt = ((current_pan < -M_PI_2) || (current_pan > M_PI_2))
                             ? std::abs(current_pan - std::copysignf(M_PI, current_pan))
                             : std::abs(current_pan);
    tilt_bound_.upper = tilt_uparams_[0] * std::exp(-tilt_uparams_[1] * std::pow(projected_tilt, 2)) + tilt_uparams_[2];
    tilt_bound_.lower = tilt_lparams_[0] * std::exp(-tilt_lparams_[1] * std::pow(projected_tilt, 2)) + tilt_lparams_[2];

    float requested_pan = 0.;
    float requested_tilt = 0.;

    if (static_cast<HeadMotionType>(current_motion_request_msg_.head_motion_request.head_motion_type) ==
        HeadMotionType::TARGET) {
      Eigen::Vector3f target = {static_cast<float>(current_motion_request_msg_.head_motion_request.target.x),
                                static_cast<float>(current_motion_request_msg_.head_motion_request.target.y),
                                static_cast<float>(current_motion_request_msg_.head_motion_request.target.z)};

      // Convert normalized target vector to pan and tilt
      target.normalize();
      requested_pan = std::atan2(target.y(), target.x());
      requested_tilt = -std::asin(target.z());

    } else {
      requested_pan = current_motion_request_msg_.head_motion_request.pan;
      requested_tilt = current_motion_request_msg_.head_motion_request.tilt;
    }

    // Set the angles to [-pi, pi] and clamp to bounds
    requested_pan -= 2 * M_PI * std::floor((requested_pan + M_PI) * (1. / (2 * M_PI)));
    requested_tilt -= 2 * M_PI * std::floor((requested_tilt + M_PI) * (1. / (2 * M_PI)));
    requested_pan = pan_bound_.clamp(requested_pan);
    requested_tilt = tilt_bound_.clamp(requested_tilt);

    // P-Control
    float error_pan = requested_pan - current_pan;
    float error_tilt = requested_tilt - current_tilt;

    if (std::abs(error_pan) < error_pan_tolerance_ && std::abs(error_tilt) < error_tilt_tolerance_) {
      is_requested_target_reached_ = true;
    } else {
      is_requested_target_reached_ = false;
    }

    float u_pan = p_pan_ * error_pan;
    float u_tilt = p_tilt_ * error_tilt;
    float requested_speed = static_cast<float>(current_motion_request_msg_.head_motion_request.speed);
    auto u_bound = Bound(std::max(max_u_bound_.lower, -requested_speed), std::min(max_u_bound_.upper, requested_speed));
    // Limit the "speed"
    u_pan = u_bound.clamp(u_pan);
    u_tilt = u_bound.clamp(u_tilt);

    float next_pan = current_pan + u_pan;
    float next_tilt = current_tilt + u_tilt;

    next_pan = pan_bound_.clamp(next_pan);
    next_tilt = tilt_bound_.clamp(next_tilt);

    current_joint_requests_.positions.values[nomadz_definitions::joint_indexes::HEAD_YAW] = next_pan;
    current_joint_requests_.positions.values[nomadz_definitions::joint_indexes::HEAD_PITCH] = next_tilt;
  }

  void HeadEngine::updateMotionInfoMsg() {
    motion_info_msg_.is_head_motion_done = is_requested_target_reached_;
    motion_info_msg_.executed_motion_request.head_motion_request = current_motion_request_msg_.head_motion_request;
  }

  bool HeadEngine::isLeavingPossible() {
    return true;
  }

  void HeadEngine::reset() {}

  void HeadEngine::initParameters() {
    auto params = param_listener_.get_params();
    p_pan_ = static_cast<float>(params.p_pan);
    p_tilt_ = static_cast<float>(params.p_tilt);
    error_pan_tolerance_ = static_cast<float>(params.error_pan_tolerance);
    error_tilt_tolerance_ = static_cast<float>(params.error_tilt_tolerance);
    pan_bound_ = Bound(static_cast<float>(params.pan_bound[0]), static_cast<float>(params.pan_bound[1]));
    tilt_bound_ = Bound(static_cast<float>(params.tilt_bound[0]), static_cast<float>(params.tilt_bound[3]));
    max_u_bound_ = Bound(static_cast<float>(params.max_u_bound[0]), static_cast<float>(params.max_u_bound[1]));
    current_joint_requests_.stiffnesses.values[nomadz_definitions::joint_indexes::HEAD_PITCH] =
      static_cast<float>(params.stiffness[0]);
    current_joint_requests_.stiffnesses.values[nomadz_definitions::joint_indexes::HEAD_YAW] =
      static_cast<float>(params.stiffness[1]);

    // Tilt limit function parameters init
    float lower_max = static_cast<float>(params.tilt_bound[0]);
    float lower_min = static_cast<float>(params.tilt_bound[1]);
    float stretch = static_cast<float>(params.tilt_bound[2]);
    float upper_min = static_cast<float>(params.tilt_bound[3]);
    float upper_max = static_cast<float>(params.tilt_bound[4]);
    float exp_const = std::exp(-stretch * std::pow(M_PI_2, 2));

    tilt_lparams_[0] = (lower_max - lower_min) / (1.0 - exp_const);
    tilt_lparams_[1] = stretch;
    tilt_lparams_[2] = (lower_min - exp_const * lower_max) / (1.0 - exp_const);

    tilt_uparams_[0] = (upper_max - upper_min) / (1.0 - exp_const);
    tilt_uparams_[1] = stretch;
    tilt_uparams_[2] = (upper_min - exp_const * upper_max) / (1.0 - exp_const);
  }
} // namespace nomadz_motion_control
