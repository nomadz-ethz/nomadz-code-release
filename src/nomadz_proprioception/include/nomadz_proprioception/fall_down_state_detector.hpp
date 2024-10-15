#pragma once

#include <chrono>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <boost/circular_buffer.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "nomadz_proprioception_msgs/msg/fall_down_state.hpp"
#include "nomadz_proprioception_msgs/fall_down_state_enums.hpp"

namespace nomadz_proprioception {
  class FallDownStateDetector {
  public:
    FallDownStateDetector();

    void updateFallDownState(bool has_ground_contact);
    void updateGravityVectorAngle(geometry_msgs::msg::Vector3 acc);
    void setAngle(float angle_x, float angle_y);

    void reset();

    nomadz_proprioception_msgs::FallDownState getFallDownState() const;
    nomadz_proprioception_msgs::FallDirection getFallDownDirection() const;

  private:
    static constexpr float STAGGERING_ANGLE_X = 15.F / 180.F * M_PI;
    static constexpr float STAGGERING_ANGLE_Y_FRONT = 18.F / 180.F * M_PI;
    static constexpr float STAGGERING_ANGLE_Y_BACK = 13.F / 180.F * M_PI;
    static constexpr float ON_GROUND_ANGLE = 75.F / 180.F * M_PI;
    static constexpr float STAGGERING_ZONE = 6.F / 180.F * M_PI;

    static constexpr int FALL_TIME = 1000;

    static constexpr int BUFFER_LEN = 15;

    bool isFalling() const;
    bool isStaggering() const;
    nomadz_proprioception_msgs::FallDirection getFallDirection() const;

    nomadz_proprioception_msgs::FallDownState fall_down_state_{nomadz_proprioception_msgs::FallDownState::UPRIGHT};
    nomadz_proprioception_msgs::FallDirection fall_direction_{nomadz_proprioception_msgs::FallDirection::NONE};

    std::chrono::steady_clock::time_point last_fall_time_;

    float angle_x_;
    float angle_y_;

    float acceleration_angle_xz_;
    float acceleration_angle_yz_;

    boost::circular_buffer<float> acc_buffers_x_{BUFFER_LEN};
    boost::circular_buffer<float> acc_buffers_y_{BUFFER_LEN};
    boost::circular_buffer<float> acc_buffers_z_{BUFFER_LEN};
  };
} // namespace nomadz_proprioception
