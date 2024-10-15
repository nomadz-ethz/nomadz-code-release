#pragma once

#include <Eigen/Geometry>
#include <boost/circular_buffer.hpp>

#include "nomadz_core/math/unscented_kalman_filter.hpp"

namespace nomadz_proprioception {

  enum class MotionPhase : std::uint8_t { STAND, WALK, OTHERS };

  class SensorDataFilter {
  public:
    SensorDataFilter();

    void processAccelerometer(const Eigen::Vector3f& acc,
                              const MotionPhase& motion_phase = MotionPhase::STAND,
                              const bool& ground_contact = true);

    void processGyroscope(const Eigen::Vector3f& gyro);

    const Eigen::Quaternionf& orientation() const { return ukf_.mean.orientation; }

    float gravity() const { return gravity_; }

  private:
    static constexpr float GYRO_VAR = 0.00000387F;
    static constexpr float ACC_VAR = 0.0221734668F;
    static constexpr float ACC_VAR_WALK = 0.036955778F;
    static constexpr float ACC_VAR_STAND = 0.018477889F;
    static constexpr float ACC_VAR_FACTOR = 3.0F;
    static constexpr float MAX_MEAN_SQUARED_DEVIATION = 0.1F;

    struct State : public nomadz_core::Manifold<3> {
      Eigen::Quaternionf orientation; // NOLINT(misc-non-private-member-variables-in-classes)

      explicit State(Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity())
          : orientation(std::move(orientation)) {}

      State operator+(const Eigen::Vector3f& angle_axis) const;
      State& operator+=(const Eigen::Vector3f& angle_axis);
      Eigen::Vector3f operator-(const State& other) const;
    };

    nomadz_core::UKFM<State> ukf_{State()};

    float gravity_ = -nomadz_core::constants::G;

    boost::circular_buffer<float> deviation10_{10};
    boost::circular_buffer<float> deviation50_{50};
    boost::circular_buffer<float> accelerometer_buffer_{50};
  };

} // namespace nomadz_proprioception
