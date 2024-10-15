#include "nomadz_proprioception/fsr_calibrator.hpp"

namespace side = nomadz_definitions::side;

namespace nomadz_proprioception {

  FsrCalibrator::FsrCalibrator() {
    for (int i = 0; i < side::NUM_SIDES; i++) {
      for (int j = 0; j < Fsr::NUM_FSRS; j++) {
        highest_pressure_[i][j] = MIN_PRESSURE;
        lowest_pressure_[i][j] = MAX_PRESSURE;
      }
    }
  }

  void FsrCalibrator::calibrate() {
    // Robot should be able to walk first
    is_calibrated_ = true;
    min_standing_pressure_ = MIN_PRESSURE_PERCENTAGE;
  }

  bool FsrCalibrator::isCalibrated() const {
    return is_calibrated_;
  }

  float FsrCalibrator::getMinStandingPressure() const {
    return min_standing_pressure_;
  }

  FsrArray FsrCalibrator::update(FsrArray current_fsr_measurements) {
    FsrArray relative_pressure;
    for (int i = 0; i < side::NUM_SIDES; i++) {
      for (int j = 0; j < Fsr::NUM_FSRS; j++) {
        highest_pressure_[i][j] = std::max(highest_pressure_[i][j], current_fsr_measurements[i][j]);
        lowest_pressure_[i][j] = std::min(lowest_pressure_[i][j], current_fsr_measurements[i][j]);

        float pressure = std::max(0.F, std::min(MAX_PRESSURE, current_fsr_measurements[i][j]) - lowest_pressure_[i][j]);
        foot_pressure_[i] += pressure;
        relative_pressure[i][j] = pressure / highest_pressure_[i][j];
      }
    }
    return relative_pressure;
  }

} // namespace nomadz_proprioception
