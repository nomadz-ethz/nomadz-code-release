#include "nomadz_motion_control/walk_engine/walk_engine.hpp"

#include <vector>
#include <tf2_eigen/tf2_eigen.hpp>

#include "nomadz_motion_control_msgs/msg/motion_request.hpp"
#include "nomadz_motion_control_msgs/msg/walk_request.hpp"
#include "nomadz_core/geometry/ros_conversion.hpp"
#include "nomadz_core/geometry/projection.hpp"
#include "nomadz_motion_control_msgs/walk_request_enums.hpp"

namespace side = nomadz_definitions::side;
using Twist2D = nomadz_core::Twist2D;
using Pose2D = nomadz_core::Pose2D;

namespace nomadz_motion_control {
  WalkEngine::WalkEngine(const rclcpp::NodeOptions& options)
      : MotionBase("walk_engine", options), param_listener_(get_node_parameters_interface()),
        dyn_params_(std::make_shared<WalkParamT>(param_listener_.get_params())), step_planner_(std::move(dyn_params_)),
        walk_generator_(std::move(dyn_params_)) {
    walk_height_ = 0.26F;

    world_model_sub_ =
      this->create_subscription<WorldModelMsgT>("modeling/world_model", 1, [this](WorldModelMsgT::ConstSharedPtr msg) {
        ball_position_.x() = static_cast<float>(msg->ball_model.position.x);
        ball_position_.y() = static_cast<float>(msg->ball_model.position.y);
        step_planner_.setBallPos(ball_position_);
      });

    orientation_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "proprioception/orientation", 1, [this](geometry_msgs::msg::QuaternionStamped::ConstSharedPtr msg) {
        Eigen::Quaterniond orientation_quat;
        tf2::fromMsg(msg->quaternion, orientation_quat);
        walk_core_state_.orientation = Eigen::Affine3f::Identity();
        walk_core_state_.orientation.linear() = orientation_quat.cast<float>().toRotationMatrix();
      });

    foot_support_sub_ = this->create_subscription<FootSupportMsgT>(
      "proprioception/foot_support", 1, [this](FootSupportMsgT::ConstSharedPtr msg) {
        force_zero_speed_ = (!msg->ground_contacts[0] && !msg->ground_contacts[1]);
        foot_support_ = *msg;
        walk_core_state_.foot_support = msg->support;
        walk_core_state_.ground_contacts[side::LEFT] = msg->ground_contacts[side::LEFT];
        walk_core_state_.ground_contacts[side::RIGHT] = msg->ground_contacts[side::RIGHT];
      });

    imu_sub_ = this->create_subscription<ImuMsgT>("sensors/imu", 1, [this](ImuMsgT::ConstSharedPtr msg) {
      walk_core_state_.angle_pitch = msg->angle_pitch;
      walk_core_state_.angle_roll = msg->angle_roll;
      walk_core_state_.gyro = nomadz_core::unpackVector3(msg->gyroscope);
    });

    measured_robot_model_sub_ = this->create_subscription<RobotModelMsgT>(
      "proprioception/measured_robot_model", 1, [this](RobotModelMsgT::ConstSharedPtr msg) {
        Eigen::Affine3d foot_poses[side::NUM_SIDES];
        tf2::fromMsg(msg->foot_poses[side::LEFT], foot_poses[side::LEFT]);
        tf2::fromMsg(msg->foot_poses[side::RIGHT], foot_poses[side::RIGHT]);
        walk_core_state_.measured_foot_poses[side::LEFT] = foot_poses[side::LEFT].cast<float>();
        walk_core_state_.measured_foot_poses[side::RIGHT] = foot_poses[side::RIGHT].cast<float>();
      });
  }

  void WalkEngine::computeNextStep() {
    step_planner_.calcStepPattern(walk_core_state_);
    if (step_planner_.getPlannedSteps().is_current_step_kick) {
      walk_core_state_.step_duration = step_planner_.getPlannedSteps().step_duration * 0.7F;
    } else {
      walk_core_state_.step_duration = step_planner_.getPlannedSteps().step_duration;
    }
    walk_core_state_.target_foot_poses[side::LEFT] =
      nomadz_core::expandTo3D(nomadz_core::toAffine2(step_planner_.getPlannedSteps().target_foot_poses[side::LEFT]) *
                                toAffine2(Pose2D{-static_cast<float>(dyn_params_->foot_origin_offset_x), 0.F, 0.F}),
                              -walk_height_);
    walk_core_state_.target_foot_poses[side::RIGHT] =
      nomadz_core::expandTo3D(nomadz_core::toAffine2(step_planner_.getPlannedSteps().target_foot_poses[side::RIGHT]) *
                                toAffine2(Pose2D{-static_cast<float>(dyn_params_->foot_origin_offset_x), 0.F, 0.F}),
                              -walk_height_);
  }

  void WalkEngine::preProcessWalkRequest(const WalkRequestMsgT& request) {
    walk_core_state_.walk_mode = static_cast<WalkMode>(request.walk_mode);
    if (walk_core_state_.walk_mode == WalkMode::SPEED) {
      if (force_zero_speed_) {
        walk_core_state_.target_speed = Twist2D{};
      } else {
        walk_core_state_.target_speed = nomadz_core::unpackTwist2D(request.target_speed);
      }
    } else {
      walk_core_state_.target_speed = Twist2D{};
      walk_core_state_.step_pattern_type = static_cast<PatternType>(request.pattern_type);
    }
    if (request_leave_) {
      walk_core_state_.walk_mode = WalkMode::SPEED;
      walk_core_state_.target_speed = Twist2D{};
    }
    if (walk_core_state_.walk_state != WalkState::WALKING && walk_core_state_.target_speed.y != 0.F) {
      walk_core_state_.is_left_phase = walk_core_state_.target_speed.y < 0.F;
    }
  }

  void WalkEngine::preProcessKickRequest(const KickRequestMsgT& request) {
    if (walk_core_state_.walk_mode == WalkMode::PATTERN) {
      step_planner_.setKickRequest(request.kick_direction, request.kick_power);
    }
    if (step_planner_.getPlannedSteps().is_current_step_kick) {
      walk_generator_.updateSwingControlPoint(step_planner_.getPlannedSteps().planned_speed, request.kick_power);
    } else {
      walk_generator_.updateSwingControlPoint(step_planner_.getPlannedSteps().planned_speed);
    }
  }

  void WalkEngine::updateJointRequests() {
    if (param_listener_.is_old(*dyn_params_)) {
      *dyn_params_ = param_listener_.get_params();
    }
    if (walk_core_state_.phase_time == 0.F) {
      preProcessWalkRequest(current_motion_request_msg_.walk_request);
      computeNextStep();
      preProcessKickRequest(current_motion_request_msg_.kick_request);
      stateCheck();
    }
    walk_generator_.generateJointRequests(walk_core_state_);
    current_joint_requests_ = walk_generator_.getWalkGeneratorData().joint_request;

    walk_core_state_.phase_time += nomadz_core::constants::MOTION_CYCLE_TIME;
    checkSwitchConditions();
  }

  void WalkEngine::checkSwitchConditions() {
    const float phase_ratio = walk_core_state_.phase_time / walk_core_state_.step_duration;
    if (phase_ratio < static_cast<float>(dyn_params_->switch_threshold_min)) {
      return;
    }
#ifdef TARGET_ROBOT
    if (foot_support_.switched || phase_ratio > SWITCH_THRESHOLD_MAX) {
      walk_core_state_.is_left_phase = !walk_core_state_.is_left_phase; // foot_support_.support < 0.F;
      walk_core_state_.phase_time = 0.F;
    } else if (foot_support_.predicted_switched) {
      walk_core_state_.is_left_phase = !walk_core_state_.is_left_phase;
      walk_core_state_.phase_time = 0.F;
    }
#else
    if (phase_ratio > 1.F) {
      walk_core_state_.is_left_phase = !walk_core_state_.is_left_phase;
      walk_core_state_.phase_time = 0.F;
    }
#endif
  }

  void WalkEngine::stateCheck() {
    Twist2D planned_speed = step_planner_.getPlannedSteps().planned_speed;
    switch (walk_core_state_.walk_state) {
    case WalkState::STANDING:
      if (planned_speed != Twist2D{}) {
        walk_core_state_.walk_state = WalkState::STARTING;
      }
      walk_core_state_.phase_time = 0.F;
      break;
    case WalkState::STARTING:
      walk_core_state_.walk_state = WalkState::WALKING;
      break;

    case WalkState::WALKING:
      if (planned_speed == Twist2D{}) {
        walk_core_state_.walk_state = WalkState::STOPPING;
      }
      break;

    case WalkState::STOPPING:
      if (foot_support_.ground_contacts[side::LEFT] && foot_support_.ground_contacts[side::RIGHT]) {
        walk_core_state_.walk_state = WalkState::STANDING;
      }
      break;

    default:
      break;
    }
  }

  void WalkEngine::updateMotionInfoMsg() {
    motion_info_msg_ = MotionInfoMsgT();
    motion_info_msg_.odometry_offset = nomadz_core::packTwist2D(step_planner_.getPlannedSteps().measured_odometry);
    motion_info_msg_.is_leaving_possible = walk_core_state_.walk_state == WalkState::STANDING;
    if (walk_core_state_.walk_mode == WalkMode::PATTERN) {
      motion_info_msg_.is_motion_done = step_planner_.getExecutingTargetPattern();
    } else {
      motion_info_msg_.is_motion_done = false;
    }
    motion_info_msg_.executed_motion_request = current_motion_request_msg_;
  }

  bool WalkEngine::isLeavingPossible() {
    bool is_leaving_possible = walk_core_state_.walk_state == WalkState::STANDING;
    if (is_leaving_possible) {
      request_leave_ = false;
    }
    return is_leaving_possible;
  }

  void WalkEngine::reset() {
    walk_core_state_ = WalkCoreState{};
    walk_core_state_.walk_state = WalkState::STANDING;
    walk_core_state_.phase_time = 0.0;
    walk_core_state_.is_left_phase = true;
    request_leave_ = false;
    step_planner_.reset();
  }
} // namespace nomadz_motion_control
