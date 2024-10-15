#pragma once

#include <cmath>
#include <Eigen/Core>

#include "nomadz_core/math/unscented_kalman_filter.hpp"
#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_modeling/landmark_registrator.hpp"
#include "nomadz_modeling/line_registrator.hpp"
#include "nomadz_modeling/pose_registrator.hpp"
#include "nomadz_modeling/self_locator_utils.hpp"

namespace nomadz_modeling {
  /**
   * @brief Represents a particle used in a UKF-based localization system.
   *
   * Each Particle encapsulates a UKF hypothesis with associated validity and weight,
   * and provides methods for odometry prediction and measurement update
   */
  class Particle {
  public:
    Particle(const Eigen::Vector3f& init_state, const Eigen::Matrix3f& init_cov);
    Particle(const Particle& other) = default;

    /**
     * The prediction step to propagate the UKF hypothesis using odometry information.
     * @param odometry_offset The odometry offset obtained from the motion pipeline
     */
    void predict(const Eigen::Vector3f& odometry_offset);

    /**
     * The update step of the UKF hypothesis given a compatible landmark measurement on the field.
     * @param landmark The compatible measured landmark on the field
     */
    void updateByLandmark(const RegisteredLandmark& landmark);

    /**
     * The update step of the UKF hypothesis given a compatible robot pose measurement on the field.
     * @param robot_pose The compatible measured robot pose on the field
     */
    void updateByRobotPose(const RegisteredRobotPose& robot_pose);

    /**
     * The update step of the UKF hypothesis given a compatible line measurement on the field.
     * @param line The compatible measured line on the field
     */
    void updateByLine(const RegisteredLine& line);

    nomadz_core::Pose2D getPose() const { return nomadz_core::toPose2D(ukf_.mean); }
    float getValidity() const { return validity_; }
    float getWeight() const { return weight_; }

    /**
     * Calculates the combined variance of the UKF hypothesis to estimate robot pose.
     * @return The combined variance
     */
    float getCombinedVariance() const { return std::max(ukf_.cov(0, 0), ukf_.cov(1, 1)) * ukf_.cov(2, 2); }

    /**
     * Updates the validity measure based on new validity information from measurements.
     * @param frames Number of frames used for averaging
     * @param new_validity The new validity measurement from field observations
     */
    void updateValidity(const int frames, const float new_validity) {
      validity_ = (validity_ * static_cast<float>(frames - 1) + new_validity) / static_cast<float>(frames);
    }

    /**
     * Updates the weight of the hypothesis based on the current validity measure.
     * @param base_validity_weight The base weight given zero validity
     */
    void updateWeight(const float base_validity_weight) {
      weight_ = base_validity_weight + (1.F - base_validity_weight) * validity_;
    }

    void invalidate() { validity_ = 0.F; }

    Particle& operator=(const Particle& other) {
      if (this != &other) {
        this->ukf_ = other.ukf_;
        this->validity_ = other.validity_;
        this->weight_ = other.weight_;
      }
      return *this;
    }

  private:
    const Eigen::Vector3f FILTER_PROCESS_DEVIATION_{0.002F, 0.002F, 0.002F};
    const Eigen::Vector3f ODOMETRY_DEVIATION_{0.0002F, 0.0002F, 0.3F};
    const Eigen::Vector2f ODOMETRY_ROTATION_DEVIATION_{0.00157F, 0.00157F};

    nomadz_core::UKF<3> ukf_;
    float validity_ = 0.5F;
    float weight_;
  };

} // namespace nomadz_modeling
