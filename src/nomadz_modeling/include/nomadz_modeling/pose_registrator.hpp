#pragma once

#include <optional>
#include <tuple>
#include <vector>

#include <Eigen/Core>

#include "nomadz_core/geometry/pose.hpp"

namespace nomadz_modeling {

  struct PerceivedFeaturePose {
    /**
     * The relative position of the perceived feature in the robot frame together
     * with the relative orientation of a vertical line
     **/
    Eigen::Vector3f percept;
    /**
     * The absolute position of the perceived feature in the world frame together
     * with zero element absolute orientation for the vertical line
     **/
    Eigen::Vector3f model;
    // The covariance of the pose measurement
    Eigen::Matrix3f cov_percept;
  };

  struct RegisteredRobotPose {
    // The perceived robot pose in the world frame
    nomadz_core::Pose2D percept;
    // The covariance of the pose measurement
    Eigen::Matrix3f cov_percept;
  };

  class PoseRegistrator {

  public:
    PoseRegistrator() = default;

    /**
     * Register the perceived feature poses based on the assumed robot pose
     * @param robot_pose The assumed robot pose
     * @param perceived_feature_poses A list containing all the perceived feature poses to analyze
     * @param registered_robot_poses A list to which all compatible robot pose measurements are added
     */
    void registerPoses(const nomadz_core::Pose2D& robot_pose,
                       const std::vector<PerceivedFeaturePose>& perceived_feature_poses,
                       std::vector<RegisteredRobotPose>& registered_robot_poses) const;

  private:
    // Maximum allowed distance error for pose registration
    const float MAX_DISTANCE_ERROR_ = 2.5F;
    // Maximum allowed angle error for pose registration
    const float MAX_ANGLE_ERROR_ = 0.7F;
    // Cov noise added after registration Note(emilio): set to 0 once the measurements have accurate cov
    const Eigen::Matrix3f ADDITIONAL_NOISE_ = 0.05F * Eigen::Matrix3f::Identity();

    /**
     * Determine if the perceived feature pose matches one of the poses on the field
     * @param robot_pose The assumed pose of the robot
     * @param perceived_feature_pose The perceived feature pose in the robot frame
     * @return An optional containing the corresponding robot pose in the world model
     */
    std::optional<nomadz_core::Pose2D>
    getCorrespondingPoseInWorldModel(const nomadz_core::Pose2D& robot_pose,
                                     const PerceivedFeaturePose& perceived_feature_pose) const;

    /**
     * Get the world robot pose and it's mirrored version in the opponent field from the perceived feature pose
     * @param perceived_feature_pose The perceived feature pose
     * @return The corresponding world robot poses
     */
    static std::tuple<nomadz_core::Pose2D, nomadz_core::Pose2D>
    getWorldRobotPosFromFeature(const PerceivedFeaturePose& perceived_feature_pose);
  };
} // namespace nomadz_modeling
