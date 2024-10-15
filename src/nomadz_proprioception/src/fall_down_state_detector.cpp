#include "nomadz_proprioception/fall_down_state_detector.hpp"

#include <cmath>
#include <numeric>

using FallDownState = nomadz_proprioception_msgs::FallDownState;
using FallDirection = nomadz_proprioception_msgs::FallDirection;

namespace nomadz_proprioception {

  FallDownStateDetector::FallDownStateDetector() {
    reset();
  }

  void FallDownStateDetector::reset() {
    fall_down_state_ = FallDownState::UNDEFINED;
    fall_direction_ = FallDirection::NONE;
    last_fall_time_ = std::chrono::steady_clock::now();
    angle_x_ = 0.F;
    angle_y_ = 0.F;
    acceleration_angle_xz_ = 0.F;
    acceleration_angle_yz_ = 0.F;
    std::fill(acc_buffers_x_.begin(), acc_buffers_x_.end(), 0);
    std::fill(acc_buffers_y_.begin(), acc_buffers_y_.end(), 0);
    std::fill(acc_buffers_z_.begin(), acc_buffers_z_.end(), 0);
  }

  void FallDownStateDetector::setAngle(float angle_x, float angle_y) {
    angle_x_ = angle_x;
    angle_y_ = angle_y;
  }

  void FallDownStateDetector::updateGravityVectorAngle(geometry_msgs::msg::Vector3 acc) {
    acc_buffers_x_.push_front(-static_cast<float>(acc.x));
    acc_buffers_y_.push_front(-static_cast<float>(acc.y));
    acc_buffers_z_.push_front(-static_cast<float>(acc.z));
    const float acc_x_mean =
      std::accumulate(acc_buffers_x_.begin(), acc_buffers_x_.end(), 0.F) / static_cast<float>(acc_buffers_x_.size());
    const float acc_y_mean =
      std::accumulate(acc_buffers_y_.begin(), acc_buffers_y_.end(), 0.F) / static_cast<float>(acc_buffers_y_.size());
    const float acc_z_mean =
      std::accumulate(acc_buffers_z_.begin(), acc_buffers_z_.end(), 0.F) / static_cast<float>(acc_buffers_z_.size());
    acceleration_angle_xz_ = std::atan2(-acc_z_mean, -acc_x_mean);
    acceleration_angle_yz_ = std::atan2(-acc_z_mean, acc_y_mean);
  }

  bool FallDownStateDetector::isFalling() const {
    return (std::abs(angle_x_) > (STAGGERING_ANGLE_X + STAGGERING_ZONE)) ||
           (angle_y_ > (STAGGERING_ANGLE_Y_FRONT + STAGGERING_ZONE)) ||
           (angle_y_ < -(STAGGERING_ANGLE_Y_BACK + STAGGERING_ZONE));
  }

  bool FallDownStateDetector::isStaggering() const {
    return (std::abs(angle_x_) > (STAGGERING_ANGLE_X)) || (angle_y_ > (STAGGERING_ANGLE_Y_FRONT)) ||
           (angle_y_ < -(STAGGERING_ANGLE_Y_BACK));
  }

  FallDirection FallDownStateDetector::getFallDirection() const {
    FallDirection fall_direction = FallDirection::NONE;
    if (std::abs(angle_x_) > std::abs(angle_y_) + 0.2F) {
      if (angle_x_ < 0.F) {
        fall_direction = FallDirection::LEFT;
      } else {
        fall_direction = FallDirection::RIGHT;
      }
    } else {
      if (angle_y_ > 0.F) {
        fall_direction = FallDirection::FRONT;
      } else {
        fall_direction = FallDirection::BACK;
      }
    }
    return fall_direction;
  }

  void FallDownStateDetector::updateFallDownState(const bool has_ground_contact) {
    // Update the fall down state
    auto current_time = std::chrono::steady_clock::now();
    switch (fall_down_state_) {
    case FallDownState::UNDEFINED:
      if (!isStaggering()) {
        fall_down_state_ = FallDownState::UPRIGHT;
        break;
      } else {
        fall_down_state_ = FallDownState::ON_GROUND;
      }
      if (std::abs(acceleration_angle_xz_) < 0.5F) {
        fall_direction_ = FallDirection::FRONT;
      } else if (std::abs(acceleration_angle_xz_) > 2.5F) {
        fall_direction_ = FallDirection::BACK;
      } else if (std::abs(acceleration_angle_yz_) < 0.5F) {
        fall_direction_ = FallDirection::LEFT;
      } else if (std::abs(acceleration_angle_yz_) > 2.5F) {
        fall_direction_ = FallDirection::RIGHT;
      } else {
        fall_down_state_ = FallDownState::UNDEFINED;
      }
      break;

    case FallDownState::UPRIGHT:
      fall_direction_ = FallDirection::NONE;
      if (isStaggering()) {
        fall_down_state_ = FallDownState::STAGGERING;
      }
      break;

    case FallDownState::STAGGERING:
      fall_direction_ = getFallDirection();
      if (isFalling() && has_ground_contact) {
        fall_down_state_ = FallDownState::FALLING;
        last_fall_time_ = std::chrono::steady_clock::now();
      } else if (!isStaggering()) {
        fall_down_state_ = FallDownState::UPRIGHT;
      }
      break;

    case FallDownState::FALLING:
      fall_direction_ = getFallDirection();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_fall_time_).count() > FALL_TIME) {
        if (!isStaggering()) {
          fall_down_state_ = FallDownState::UPRIGHT;
        } else {
          fall_down_state_ = FallDownState::UNDEFINED;
        }
      }
      break;

    case FallDownState::ON_GROUND:
      if (!isStaggering()) {
        fall_down_state_ = FallDownState::UPRIGHT;
      }
      if (fall_direction_ != FallDirection::FRONT && fall_direction_ != FallDirection::BACK) {
        fall_direction_ = getFallDirection();
      }
      break;
    }
  }

  FallDownState FallDownStateDetector::getFallDownState() const {
    return fall_down_state_;
  }
  FallDirection FallDownStateDetector::getFallDownDirection() const {
    return fall_direction_;
  }

} // namespace nomadz_proprioception
