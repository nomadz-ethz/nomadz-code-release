#include "nomadz_motion_control/interpolation.hpp"

#include <cmath>
#include <algorithm>
#include <functional>

#include "nomadz_core/math/constants.hpp"

namespace nomadz_motion_control {
  void JointRequestInterpolator::process(JointRequests& joint_request, bool should_interpolate) {

    const bool interpolation_in_progress = interpolant_ < 1.0F;
    if (!should_interpolate && !interpolation_in_progress) {
      previous_joint_requests_ = joint_request;
      return;
    }

    // Start the interpolation process
    if (should_interpolate && !interpolation_in_progress) {
      interpolant_ = 0.0F;
    }

    interpolant_ = std::min(interpolant_ + nomadz_core::constants::MOTION_CYCLE_TIME / INTERPOLATION_TIME, 1.0F);

    const bool interpolation_finished = std::abs(interpolant_ - 1.0F) < 1e-6F;
    if (interpolation_finished) {
      previous_joint_requests_ = joint_request;
      return;
    }

    JointRequests interpolated_joint_requests = linearInterpolation(previous_joint_requests_, joint_request, interpolant_);

    // create interpolated mask by using OR
    std::transform(joint_request.joint_ignore.begin(),
                   joint_request.joint_ignore.end(),
                   previous_joint_requests_.joint_ignore.begin(),
                   interpolated_joint_requests.joint_ignore.begin(),
                   std::logical_or<bool>());

    joint_request = interpolated_joint_requests;
  }
} // namespace nomadz_motion_control
