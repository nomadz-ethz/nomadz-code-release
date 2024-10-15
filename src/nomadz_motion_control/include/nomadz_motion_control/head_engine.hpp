#pragma once

#include <Eigen/Core>

#include "nomadz_motion_control/motion_base.hpp"
#include "head_engine_parameters.hpp"

namespace nomadz_motion_control {

  struct Bound {
    float lower;
    float upper;
    Bound() : lower(0.0), upper(0.0) {}
    Bound(float lower, float upper) : lower(lower), upper(upper) {}
    float clamp(const float value) const { return std::clamp(value, this->lower, this->upper); }
  };

  class HeadEngine : public MotionBase {
  public:
    explicit HeadEngine(const rclcpp::NodeOptions& options);

  private:
    void updateJointRequests() override;
    void updateMotionInfoMsg() override;
    bool isLeavingPossible() override;
    void reset() override;

    void initParameters();

    head_engine_parameters::ParamListener param_listener_;

    float p_pan_ = 0.0;
    float p_tilt_ = 0.0;
    float error_pan_tolerance_ = 0.0;
    float error_tilt_tolerance_ = 0.0;

    std::vector<float> tilt_lparams_ = {0.0, 0.0, 0.0};
    std::vector<float> tilt_uparams_ = {0.0, 0.0, 0.0};

    Bound pan_bound_;
    Bound tilt_bound_;
    Bound max_u_bound_;

    bool is_requested_target_reached_{false};
  };
} // namespace nomadz_motion_control
