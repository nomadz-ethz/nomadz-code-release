#pragma once

#include <rclcpp/rclcpp.hpp>

#include "nomadz_definitions/limbs.hpp"

#include "nomadz_proprioception/types.hpp"

namespace nomadz_proprioception {

  class FsrCalibrator {
  public:
    FsrCalibrator();

    void calibrate();
    bool isCalibrated() const;
    float getMinStandingPressure() const;
    FsrArray update(FsrArray current_fsr_measurements);

  private:
    static constexpr float MIN_PRESSURE_PERCENTAGE = 0.1F;
    static constexpr float MIN_PRESSURE = 0.3F;
    static constexpr float MAX_PRESSURE = 5.0F;
    static constexpr int NUM_SUPPORT_SWITCHES = 10;

    FsrArray highest_pressure_;
    FsrArray lowest_pressure_;
    std::array<float, nomadz_definitions::side::NUM_SIDES> foot_pressure_;
    bool is_calibrated_ = false;
    float min_standing_pressure_ = 0.0F;
  };

} // namespace nomadz_proprioception
