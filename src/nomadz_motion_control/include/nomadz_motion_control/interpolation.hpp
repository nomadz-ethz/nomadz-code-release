#pragma once

#include <Eigen/Geometry>

#include "nomadz_motion_control/joint_requests.hpp"

namespace nomadz_motion_control {

  enum class InterpolationType { LINEAR, QUADRATIC_BEZIER, CUBIC_BEZIER };

  template <typename T> T inline linearInterpolation(const T& start, const T& target, float ratio) {
    return start + (target - start) * ratio;
  };

  // template specialization for type: Eigen::Affine3f
  template <>
  Eigen::Affine3f inline linearInterpolation(const Eigen::Affine3f& start, const Eigen::Affine3f& target, float ratio) {
    const Eigen::Vector3f start_position = Eigen::Vector3f{start.translation()};
    const Eigen::Vector3f target_position = Eigen::Vector3f{target.translation()};
    Eigen::Vector3f interpolated_position = linearInterpolation(start_position, target_position, ratio);

    Eigen::Quaternionf rot1(start.rotation());
    Eigen::Quaternionf rot2(target.rotation());
    Eigen::Quaternionf interpolated_rotation = rot1.slerp(ratio, rot2);

    Eigen::Affine3f result = Eigen::Affine3f::Identity();
    result.translate(interpolated_position);
    result.rotate(interpolated_rotation);
    return result;
  };

  template <typename T>
  T inline bezierInterpolation(const T& start, const T& control_point_1, const T& target, float ratio) {
    return start * (1 - ratio) * (1 - ratio) + control_point_1 * 2 * (1 - ratio) * ratio + target * ratio * ratio;
  };

  // template specialization for type: Eigen::Affine3f
  template <>
  Eigen::Affine3f inline bezierInterpolation(const Eigen::Affine3f& start,
                                             const Eigen::Affine3f& control_point_1,
                                             const Eigen::Affine3f& target,
                                             float ratio) {
    const Eigen::Vector3f start_position = Eigen::Vector3f{start.translation()};
    const Eigen::Vector3f control_point_1_position = Eigen::Vector3f{control_point_1.translation()};
    const Eigen::Vector3f target_position = Eigen::Vector3f{target.translation()};
    Eigen::Vector3f interpolated_position =
      bezierInterpolation(start_position, control_point_1_position, target_position, ratio);

    Eigen::Quaternionf rot1(start.rotation());
    Eigen::Quaternionf rot2(target.rotation());
    Eigen::Quaternionf interpolated_rotation = rot1.slerp(ratio, rot2);

    Eigen::Affine3f result = Eigen::Affine3f::Identity();
    result.translate(interpolated_position);
    result.rotate(interpolated_rotation);
    return result;
  };

  template <typename T>
  T inline bezierInterpolation(
    const T& start, const T& control_point_1, const T& control_point_2, const T& target, float ratio) {
    return start * (1 - ratio) * (1 - ratio) * (1 - ratio) + control_point_1 * 3 * (1 - ratio) * (1 - ratio) * ratio +
           control_point_2 * 3 * (1 - ratio) * ratio * ratio + target * ratio * ratio * ratio;
  };

  // template specialization for type: Eigen::Affine3f
  template <>
  Eigen::Affine3f inline bezierInterpolation(const Eigen::Affine3f& start,
                                             const Eigen::Affine3f& control_point_1,
                                             const Eigen::Affine3f& control_point_2,
                                             const Eigen::Affine3f& target,
                                             float ratio) {
    const Eigen::Vector3f start_position = Eigen::Vector3f{start.translation()};
    const Eigen::Vector3f control_point_1_position = Eigen::Vector3f{control_point_1.translation()};
    const Eigen::Vector3f control_point_2_position = Eigen::Vector3f{control_point_2.translation()};
    const Eigen::Vector3f target_position = Eigen::Vector3f{target.translation()};
    Eigen::Vector3f interpolated_position =
      bezierInterpolation(start_position, control_point_1_position, control_point_2_position, target_position, ratio);

    Eigen::Quaternionf rot1(start.rotation());
    Eigen::Quaternionf rot2(target.rotation());
    Eigen::Quaternionf interpolated_rotation = rot1.slerp(ratio, rot2);

    Eigen::Affine3f result = Eigen::Affine3f::Identity();
    result.translate(interpolated_position);
    result.rotate(interpolated_rotation);
    return result;
  };

  /**
   * @brief Interpolator to interpolate between joint requests
   *
   * Abstracts the interpolation away into one single function call so we don't need to keep track
   * of any weight or past values
   */
  class JointRequestInterpolator {
  public:
    JointRequestInterpolator() = default;

    /**
     * @brief Interpolates a joint request if necessary, otherwise leaves it unchanged.
     *
     * @param joint_request The joint request to be (potentially) interpolated
     * @param should_interpolate Whether interpolating should occur to the specified joint_request.
     *
     * Note that should_interpolate is an event based flag. I.e. iit should only be set to true at the
     * instant a switch occurs and we need to interpolate from the previous joint request to the
     * current one.
     * Internally keeps track of the last joint request so if should_interpolate is true, interpolation
     * will occur between the currently provided joint_request and the joint_request which was provided
     * in the last call before should_interpolate was set to true.
     *
     */
    void process(JointRequests& joint_request, bool should_interpolate);

  private:
    static constexpr float INTERPOLATION_TIME = 0.6F; // [s]
    JointRequests previous_joint_requests_;
    float interpolant_ = 1.0F;
  };
} // namespace nomadz_motion_control
