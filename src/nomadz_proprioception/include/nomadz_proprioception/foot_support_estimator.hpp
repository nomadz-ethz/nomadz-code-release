#pragma once

#include <chrono>

#include <Eigen/Geometry>

#include "nomadz_definitions/limbs.hpp"
#include "nomadz_proprioception/fsr_calibrator.hpp"
#include "nomadz_proprioception_msgs/msg/foot_support.hpp"

#include "nomadz_proprioception/types.hpp"

namespace nomadz_proprioception {

  class FootSupportEstimator {
  private:
    using FootSupportMsgT = nomadz_proprioception_msgs::msg::FootSupport;

  public:
    FootSupportEstimator();

    void update(const FsrArray& fsr_measurements, const Timestamp& timestamp);

    const FootSupportMsgT& getFootSupport() const { return foot_support_; }

    bool getGroundContact() { return ground_contacts_[0] || ground_contacts_[1]; }

  private:
    static constexpr float OUTER_WEIGHT = 0.8F;
    static constexpr float INNER_WEIGHT = 0.3F;
    static constexpr float CURRENT_SUPPORT_MAX_PRESSURE = 0.36F;
    static constexpr float CURRENT_SUPPORT_MIN_PRESSURE = 0.1F;

    static constexpr float FSR_CONTACT_CHANGE_TIME = 0.25F; // in seconds
    static constexpr float FSR_MIN_CONTACT_FORCE = 0.1F;

    FootSupportMsgT foot_support_;

    FsrArray weights_;
    float last_support_;
    bool had_pressure_;

    std::array<float, nomadz_definitions::side::NUM_SIDES> max_leg_fsr_measurements_;
    std::array<bool, nomadz_definitions::side::NUM_SIDES> ground_contacts_;
    std::array<std::chrono::steady_clock::time_point, nomadz_definitions::side::NUM_SIDES> last_ground_contact_time_;
    std::array<std::chrono::steady_clock::time_point, nomadz_definitions::side::NUM_SIDES> last_non_ground_contact_time_;

    FsrCalibrator fsr_calibrator_;
  };

} // namespace nomadz_proprioception
