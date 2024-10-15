#include "nomadz_proprioception/foot_support_estimator.hpp"

namespace side = nomadz_definitions::side;

namespace nomadz_proprioception {

  FootSupportEstimator::FootSupportEstimator() {
    weights_[side::LEFT][Fsr::FRONT_LEFT] = OUTER_WEIGHT;
    weights_[side::LEFT][Fsr::BACK_LEFT] = OUTER_WEIGHT;

    weights_[side::RIGHT][Fsr::FRONT_RIGHT] = -OUTER_WEIGHT;
    weights_[side::RIGHT][Fsr::BACK_RIGHT] = -OUTER_WEIGHT;

    weights_[side::LEFT][Fsr::FRONT_RIGHT] = INNER_WEIGHT;
    weights_[side::LEFT][Fsr::BACK_RIGHT] = INNER_WEIGHT;

    weights_[side::RIGHT][Fsr::FRONT_LEFT] = -INNER_WEIGHT;
    weights_[side::RIGHT][Fsr::BACK_LEFT] = -INNER_WEIGHT;

    last_support_ = 0.0F;
    fsr_calibrator_.calibrate();
  }

  void FootSupportEstimator::update(const FsrArray& fsr_measurements, const Timestamp& timestamp) {
    float total_pressure = 0.F;
    float weighted_sum = 0.F;
    const FsrArray calibrated_fsr_measurements = fsr_calibrator_.update(fsr_measurements);
    for (int i = 0; i < side::NUM_SIDES; i++) {
      for (int j = 0; j < Fsr::NUM_FSRS; j++) {
        const float weight = weights_[i][j] * calibrated_fsr_measurements[i][j];
        weighted_sum += weight;
        total_pressure += std::abs(weight);
      }
    }

    foot_support_.total_pressure = total_pressure;
    if (total_pressure > fsr_calibrator_.getMinStandingPressure()) {
      foot_support_.support = weighted_sum / total_pressure;
      foot_support_.trusted_support = fsr_calibrator_.isCalibrated();
      foot_support_.switched =
        (last_support_ * foot_support_.support < 0.F) || (!had_pressure_ && foot_support_.support != 0.F);
      float predicted_support = foot_support_.support + 3.F * (foot_support_.support - last_support_);
      foot_support_.predicted_switched = (foot_support_.support * predicted_support < 0.F) && fsr_calibrator_.isCalibrated();
      last_support_ = foot_support_.support;
      had_pressure_ = true;
    } else {
      foot_support_.support = 0.F;
      foot_support_.trusted_support = false;
      foot_support_.switched = false;
      foot_support_.predicted_switched = false;
      last_support_ = 0.F;
      had_pressure_ = false;
    }

    for (int i = 0; i < side::NUM_SIDES; i++) {
      max_leg_fsr_measurements_[i] = 0.F;
      for (auto measurement : calibrated_fsr_measurements[i]) {
        max_leg_fsr_measurements_[i] = std::max(max_leg_fsr_measurements_[i], measurement);
      }
      if (max_leg_fsr_measurements_[i] > FSR_MIN_CONTACT_FORCE) {
        last_ground_contact_time_[i] = timestamp;
      } else {
        last_non_ground_contact_time_[i] = timestamp;
      }
      if (ground_contacts_[i] &&
          std::chrono::duration<float>(timestamp - last_ground_contact_time_[i]).count() > FSR_CONTACT_CHANGE_TIME) {
        ground_contacts_[i] = false;
      }
      if (!ground_contacts_[i] &&
          std::chrono::duration<float>(timestamp - last_non_ground_contact_time_[i]).count() > FSR_CONTACT_CHANGE_TIME) {
        ground_contacts_[i] = true;
      }
      foot_support_.ground_contacts[i] = ground_contacts_[i];
    }
  }
} // namespace nomadz_proprioception
