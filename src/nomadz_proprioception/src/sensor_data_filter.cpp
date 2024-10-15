
#include "nomadz_proprioception/sensor_data_filter.hpp"

#include <numeric>

using Eigen::Vector3f;
using nomadz_core::sqr;
using nomadz_core::constants::MOTION_CYCLE_TIME;

namespace nomadz_proprioception {
  SensorDataFilter::State SensorDataFilter::State::operator+(const Vector3f& angle_axis) const {
    return State(*this) += angle_axis;
  }

  SensorDataFilter::State& SensorDataFilter::State::operator+=(const Vector3f& angle_axis) {
    orientation = orientation * Eigen::AngleAxisf(angle_axis.norm(), angle_axis.normalized());
    return *this;
  }

  Vector3f SensorDataFilter::State::operator-(const State& other) const {
    Eigen::AngleAxisf angle_axis = Eigen::AngleAxisf(other.orientation.inverse() * orientation);
    return angle_axis.axis() / angle_axis.axis().norm() * angle_axis.angle();
  }

  SensorDataFilter::SensorDataFilter() {
    Eigen::Quaternionf rot = Eigen::Quaternionf::Identity();
    ukf_.init(State(rot), Eigen::Matrix3f::Identity());
  }

  void
  SensorDataFilter::processAccelerometer(const Vector3f& acc, const MotionPhase& motion_phase, const bool& ground_contact) {
    if (motion_phase == MotionPhase::STAND && ground_contact) {
      accelerometer_buffer_.push_front(acc.norm());
      if (accelerometer_buffer_.full()) {
        const float mean = std::accumulate(accelerometer_buffer_.begin(), accelerometer_buffer_.end(), 0.F) /
                           static_cast<float>(accelerometer_buffer_.size());

        float mean_square_deviation = sqr(mean - accelerometer_buffer_[0]);
        float min = accelerometer_buffer_[0];
        float max = accelerometer_buffer_[0];
        for (size_t i = 1; i < accelerometer_buffer_.size(); ++i) {
          if (accelerometer_buffer_[i] < min) {
            min = accelerometer_buffer_[i];
          } else if (accelerometer_buffer_[i] > max) {
            max = accelerometer_buffer_[i];
          }
          mean_square_deviation += sqr(mean - accelerometer_buffer_[i]);
        }
        mean_square_deviation /= static_cast<float>(accelerometer_buffer_.size());

        if (mean_square_deviation < sqr(MAX_MEAN_SQUARED_DEVIATION) && (max - min) < 2.F * MAX_MEAN_SQUARED_DEVIATION) {
          gravity_ = mean;
        }
      }
    } else {
      accelerometer_buffer_.clear();
    }

    auto measurement_model = [&](const State& state) { return state.orientation.inverse() * Vector3f(0.F, 0.F, gravity_); };

    float use_base_acc_var = motion_phase == MotionPhase::WALK && ground_contact
                               ? ACC_VAR_WALK
                               : (motion_phase == MotionPhase::STAND ? ACC_VAR_STAND : ACC_VAR);
    float acc_len_noise = 0.F;
    if (motion_phase != MotionPhase::WALK && motion_phase != MotionPhase::STAND) {
      acc_len_noise = sqr(acc.norm() - gravity_) * ACC_VAR_FACTOR;
    }
    Vector3f measurement_noise = Vector3f(1.F, 1.F, 1.F) * (use_base_acc_var + acc_len_noise);
    ukf_.update<3>(acc, measurement_model, measurement_noise.cwiseAbs2().asDiagonal());
  }

  void SensorDataFilter::processGyroscope(const Vector3f& gyro) {
    auto dynamic_model = [&](State& state) {
      Vector3f gyro_step = gyro * MOTION_CYCLE_TIME;
      state.orientation = state.orientation * Eigen::AngleAxisf(gyro_step.norm(), gyro_step.normalized());
    };

    const float dynamic_noise = GYRO_VAR * std::sqrt(1.F / MOTION_CYCLE_TIME);
    ukf_.predict(dynamic_model, dynamic_noise * Vector3f(1.F, 1.F, 1.F).cwiseAbs2().asDiagonal());
  }
} // namespace nomadz_proprioception
