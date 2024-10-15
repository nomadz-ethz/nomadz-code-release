#include "nomadz_proprioception/arm_contact_estimator.hpp"

#include <cmath>

#include <Eigen/Core>
#include <tf2_eigen/tf2_eigen.hpp>

#include "nomadz_proprioception_msgs/msg/robot_model.hpp"
#include "nomadz_kinematics/robot_model.hpp"

namespace limbs = nomadz_definitions::limbs;
namespace side = nomadz_definitions::side;

namespace nomadz_proprioception

{

  void ArmContactEstimator::calculateError(bool is_right_side,
                                           Eigen::Affine3f& current_hand_positions,
                                           Eigen::Affine3f& requested_hand_positions) {

    const Eigen::Vector2f position_hand = current_hand_positions.translation().head<2>();
    const Eigen::Vector2f requested_hand = requested_hand_positions.translation().head<2>();
    if (is_right_side) {
      const Eigen::Vector2f right_error = position_hand - requested_hand;
      right_error_buffer_.push_front(right_error);
    } else {
      const Eigen::Vector2f left_error = position_hand - requested_hand;
      left_error_buffer_.push_front(left_error);
    }
  }

  void ArmContactEstimator::reset() {
    arm_contact_model_ = ArmContactModel();
    left_error_buffer_.clear();
    right_error_buffer_.clear();
  }

  void ArmContactEstimator::update(const nomadz_kinematics::RobotModel& measured_robot_model,
                                   const nomadz_proprioception_msgs::msg::RobotModel& requested_robot_model_msg) {

    Eigen::Affine3f left_hand_pose = measured_robot_model.getLimbs()[limbs::WRIST_LEFT];
    Eigen::Affine3f right_hand_pose = measured_robot_model.getLimbs()[limbs::WRIST_RIGHT];
    Eigen::Affine3d requested_left_hand_pose;
    Eigen::Affine3d requested_right_hand_pose;
    tf2::fromMsg(requested_robot_model_msg.hand_poses[side::LEFT], requested_left_hand_pose);
    tf2::fromMsg(requested_robot_model_msg.hand_poses[side::RIGHT], requested_right_hand_pose);
    Eigen::Affine3f requested_left_hand_pose_f = requested_left_hand_pose.cast<float>();
    Eigen::Affine3f requested_right_hand_pose_f = requested_right_hand_pose.cast<float>();

    /* Calculate the error */
    calculateError(false, left_hand_pose, requested_left_hand_pose_f);
    calculateError(true, right_hand_pose, requested_right_hand_pose_f);

    if (left_error_buffer_.full()) {
      left_ = std::accumulate(left_error_buffer_.begin(), left_error_buffer_.end(), Eigen::Vector2f::Zero().eval()) /
              static_cast<float>(left_error_buffer_.size());
    }
    if (right_error_buffer_.full()) {
      right_ = std::accumulate(right_error_buffer_.begin(), right_error_buffer_.end(), Eigen::Vector2f::Zero().eval()) /
               static_cast<float>(right_error_buffer_.size());
    }

    // update the model

    // check if the error is above a certain threshold
    bool left_x = fabs(left_(0)) > error_x_threshold_;
    bool left_y = fabs(left_(1)) > error_y_threshold_;
    bool right_x = fabs(right_(0)) > error_x_threshold_;
    bool right_y = fabs(right_(1)) > error_y_threshold_;

    arm_contact_model_.contact_left = (left_x || left_y);

    arm_contact_model_.contact_right = (right_x || right_y);

    start_time_left_ = arm_contact_model_.contact_left ? start_time_left_ : rclcpp::Clock().now();
    start_time_right_ = arm_contact_model_.contact_right ? start_time_right_ : rclcpp::Clock().now();
    arm_contact_model_.duration_left = (rclcpp::Clock().now() - start_time_left_);
    arm_contact_model_.duration_right = (rclcpp::Clock().now() - start_time_right_);

    arm_contact_model_.push_direction_left = getDirection(left_x, left_y, left_);
    arm_contact_model_.push_direction_right = getDirection(right_x, right_y, right_);
  }

  nomadz_proprioception::PushDirection
  ArmContactEstimator::getDirection(bool contact_x, bool contact_y, Eigen::Vector2f error) {

    PushDirection result = PushDirection::NONE;
    if (contact_x) {
      if (error(0) < 0.0) {
        result = PushDirection::BACKWARD;
      } else {
        result = PushDirection::FORWARD;
      }
    } else if (contact_y) {
      if (error(1) < 0.0) {
        result = PushDirection::RIGHT;
      } else {
        result = PushDirection::LEFT;
      }
    } else {
      result = PushDirection::NONE;
    }
    return result;
  }

} // namespace nomadz_proprioception
