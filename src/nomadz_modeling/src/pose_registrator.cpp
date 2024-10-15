#include "nomadz_modeling/pose_registrator.hpp"

#include <cmath>

#include "nomadz_core/math/constants.hpp"
#include "nomadz_core/geometry/angle.hpp"

namespace nomadz_modeling {

  void PoseRegistrator::registerPoses(const nomadz_core::Pose2D& robot_pose,
                                      const std::vector<PerceivedFeaturePose>& perceived_feature_poses,
                                      std::vector<RegisteredRobotPose>& registered_robot_poses) const {
    for (const PerceivedFeaturePose& perceived_feature_pose : perceived_feature_poses) {
      auto field_pose = getCorrespondingPoseInWorldModel(robot_pose, perceived_feature_pose);
      if (field_pose.has_value()) {
        RegisteredRobotPose registered_robot_pose;

        registered_robot_pose.percept = field_pose.value();

        // Update covariance matrix
        Eigen::Affine2f transformation = Eigen::Affine2f::Identity();
        transformation.rotate(field_pose.value().theta);
        registered_robot_pose.cov_percept =
          transformation.matrix() * perceived_feature_pose.cov_percept * transformation.matrix().transpose() +
          ADDITIONAL_NOISE_; // Note(emilio): remove added noise once measurement has covariance

        registered_robot_poses.push_back(registered_robot_pose);
      }
    }
  }

  std::optional<nomadz_core::Pose2D>
  PoseRegistrator::getCorrespondingPoseInWorldModel(const nomadz_core::Pose2D& robot_pose,
                                                    const PerceivedFeaturePose& perceived_feature_pose) const {
    auto poses = getWorldRobotPosFromFeature(perceived_feature_pose);
    const nomadz_core::Pose2D p = std::get<0>(poses);
    const nomadz_core::Pose2D p_mirrored = std::get<1>(poses);
    const nomadz_core::Twist2D delta_p = robot_pose - p;
    const nomadz_core::Twist2D delta_p_mirrored = robot_pose - p_mirrored;
    const float distance = (Eigen::Vector2f{delta_p.x, delta_p.y}).norm();
    const float distance_mirrored = (Eigen::Vector2f{delta_p_mirrored.x, delta_p_mirrored.y}).norm();
    float angle = std::abs(delta_p.theta);
    float angle_mirrored = std::abs(delta_p_mirrored.theta);
    nomadz_core::Pose2D model_pose;
    if (angle <= angle_mirrored && distance <= distance_mirrored) {
      model_pose.x = p.x;
      model_pose.y = p.y;
      model_pose.theta = p.theta;
    } else if (angle_mirrored <= angle && distance_mirrored < distance) {
      model_pose.x = p_mirrored.x;
      model_pose.y = p_mirrored.y;
      model_pose.theta = p_mirrored.theta;
    } else {
      return {};
    }

    Eigen::Vector3f delta_pose = nomadz_core::toVector3(robot_pose - model_pose);
    if (delta_pose.head<2>().norm() < MAX_DISTANCE_ERROR_ && delta_pose.z() < MAX_ANGLE_ERROR_) {
      return model_pose;
    }
    return {};
  }

  std::tuple<nomadz_core::Pose2D, nomadz_core::Pose2D>
  PoseRegistrator::getWorldRobotPosFromFeature(const PerceivedFeaturePose& perceived_feature_pose) {
    float delta_theta = nomadz_core::angle::normalize(perceived_feature_pose.model.z() - perceived_feature_pose.percept.z());
    Eigen::Rotation2Df rotation{delta_theta};
    Eigen::Vector2f robot_position =
      perceived_feature_pose.model.head<2>() - rotation * perceived_feature_pose.percept.head<2>();
    nomadz_core::Pose2D robot_pos{robot_position.x(), robot_position.y(), delta_theta};
    nomadz_core::Pose2D robot_pos_mirrored{
      -robot_position.x(), -robot_position.y(), nomadz_core::angle::normalize(delta_theta + nomadz_core::constants::PI)};
    return std::make_tuple(robot_pos, robot_pos_mirrored);
  }
} // namespace nomadz_modeling
