#include <gtest/gtest.h>

#include <Eigen/Core>

#include "nomadz_configuration/field_dimensions.hpp"
#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_modeling/pose_registrator.hpp"

template <int N> Eigen::Matrix3f computeCov(std::array<Eigen::Vector3f, N>& points) {
  // Compute the mean of the points
  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  for (const auto& point : points) {
    mean += point;
  }
  mean /= N;

  // Subtract the mean from each point to get mean-centered data
  Eigen::MatrixXf centered(3, N);
  for (int i = 0; i < N; ++i) {
    centered.col(i) = points[i] - mean;
  }

  // Compute the covariance matrix
  Eigen::Matrix3f covariance = (centered * centered.transpose()) / (N - 1);

  return covariance;
}

TEST(testPoseRegistration, ExactFeaturePosesExactFieldPose) {

  const nomadz_core::Pose2D test_pose{1.0, 0.0, 0.0};
  const Eigen::Affine2f test_pose_eigen = nomadz_core::toAffine2(test_pose);
  const Eigen::Affine2f test_pose_eigen_inv = test_pose_eigen.inverse();

  const nomadz_configuration::Dims dims;

  const Eigen::Vector3f penalty_mark_model = Eigen::Vector3f{dims.x_pos_opponent_penalty_area, 0.F, 0.F};
  const Eigen::Vector3f center_circle_model = Eigen::Vector3f{0.F, 0.F, 0.F};

  nomadz_modeling::PerceivedFeaturePose perceived_penalty_mark;
  Eigen::Vector2f perceived_penalty_mark_pos = test_pose_eigen_inv * penalty_mark_model.head<2>();
  perceived_penalty_mark.percept =
    Eigen::Vector3f{perceived_penalty_mark_pos.x(), perceived_penalty_mark_pos.y(), -test_pose.theta};
  perceived_penalty_mark.model = penalty_mark_model;
  perceived_penalty_mark.cov_percept = Eigen::Matrix3f::Zero();

  nomadz_modeling::PerceivedFeaturePose perceived_center_circle;
  Eigen::Vector2f perceived_center_circle_pos = test_pose_eigen_inv * center_circle_model.head<2>();
  perceived_center_circle.percept =
    Eigen::Vector3f{perceived_center_circle_pos.x(), perceived_center_circle_pos.y(), -test_pose.theta};
  perceived_center_circle.model = center_circle_model;
  perceived_center_circle.cov_percept = Eigen::Matrix3f::Zero();

  std::vector<nomadz_modeling::PerceivedFeaturePose> perceived_feature_poses;
  perceived_feature_poses.push_back(perceived_penalty_mark);
  perceived_feature_poses.push_back(perceived_center_circle);

  nomadz_modeling::PoseRegistrator pose_registrator;

  std::vector<nomadz_modeling::RegisteredRobotPose> registered_robot_poses;

  pose_registrator.registerPoses(test_pose, perceived_feature_poses, registered_robot_poses);

  EXPECT_TRUE(registered_robot_poses.size() == 2);
  for (const nomadz_modeling::RegisteredRobotPose& robot_pose : registered_robot_poses) {
    EXPECT_TRUE(nomadz_core::toVector3(robot_pose.percept).isApprox(nomadz_core::toVector3(test_pose)));
  }
}

TEST(testPoseRegistration, ExactFeaturePosesInexactFieldPose) {

  const nomadz_core::Pose2D test_pose_gt{1.0, 0.0, 0.0};
  const nomadz_core::Pose2D test_pose{0.0, 0.0, 0.0};
  const Eigen::Affine2f test_pose_eigen_gt = nomadz_core::toAffine2(test_pose_gt);
  const Eigen::Affine2f test_pose_eigen_inv_gt = test_pose_eigen_gt.inverse();

  const nomadz_configuration::Dims dims;

  const Eigen::Vector3f penalty_mark_model = Eigen::Vector3f{dims.x_pos_opponent_penalty_area, 0.F, 0.F};
  const Eigen::Vector3f center_circle_model = Eigen::Vector3f{0.F, 0.F, 0.F};

  nomadz_modeling::PerceivedFeaturePose perceived_penalty_mark;
  Eigen::Vector2f perceived_penalty_mark_pos = test_pose_eigen_inv_gt * penalty_mark_model.head<2>();
  perceived_penalty_mark.percept =
    Eigen::Vector3f{perceived_penalty_mark_pos.x(), perceived_penalty_mark_pos.y(), -test_pose_gt.theta};
  perceived_penalty_mark.model = penalty_mark_model;
  perceived_penalty_mark.cov_percept = Eigen::Matrix3f::Zero();

  nomadz_modeling::PerceivedFeaturePose perceived_center_circle;
  Eigen::Vector2f perceived_center_circle_pos = test_pose_eigen_inv_gt * center_circle_model.head<2>();
  perceived_center_circle.percept =
    Eigen::Vector3f{perceived_center_circle_pos.x(), perceived_center_circle_pos.y(), -test_pose_gt.theta};
  perceived_center_circle.model = center_circle_model;
  perceived_center_circle.cov_percept = Eigen::Matrix3f::Zero();

  std::vector<nomadz_modeling::PerceivedFeaturePose> perceived_feature_poses;
  perceived_feature_poses.push_back(perceived_penalty_mark);
  perceived_feature_poses.push_back(perceived_center_circle);

  nomadz_modeling::PoseRegistrator pose_registrator;

  std::vector<nomadz_modeling::RegisteredRobotPose> registered_robot_poses;

  pose_registrator.registerPoses(test_pose, perceived_feature_poses, registered_robot_poses);

  EXPECT_TRUE(registered_robot_poses.size() == 2);
  for (const nomadz_modeling::RegisteredRobotPose& robot_pose : registered_robot_poses) {
    EXPECT_TRUE(nomadz_core::toVector3(robot_pose.percept).isApprox(nomadz_core::toVector3(test_pose_gt)));
  }
}

TEST(testPoseRegistration, InexactFeaturePosesExactFieldPose) {

  const nomadz_core::Pose2D test_pose{1.0, 0.0, 0.0};
  const Eigen::Affine2f test_pose_eigen = nomadz_core::toAffine2(test_pose);
  const Eigen::Affine2f test_pose_eigen_inv = test_pose_eigen.inverse();

  std::array<Eigen::Vector3f, 2> test_noise = {Eigen::Vector3f{-0.00454064, 0.04921433, 0.05975387},
                                               Eigen::Vector3f{0.01455327, 0.0328405, 0.03236976}};

  Eigen::Matrix3f cov_percept = computeCov<2>(test_noise);

  const nomadz_configuration::Dims dims;

  const Eigen::Vector3f penalty_mark_model = Eigen::Vector3f{dims.x_pos_opponent_penalty_area, 0.F, 0.F};
  const Eigen::Vector3f center_circle_model = Eigen::Vector3f{0.F, 0.F, 0.F};

  nomadz_modeling::PerceivedFeaturePose perceived_penalty_mark;
  Eigen::Vector2f perceived_penalty_mark_pos = test_pose_eigen_inv * penalty_mark_model.head<2>();
  perceived_penalty_mark.percept =
    Eigen::Vector3f{perceived_penalty_mark_pos.x(), perceived_penalty_mark_pos.y(), -test_pose.theta} + test_noise[0];
  perceived_penalty_mark.model = penalty_mark_model;
  perceived_penalty_mark.cov_percept = cov_percept;

  nomadz_modeling::PerceivedFeaturePose perceived_center_circle;
  Eigen::Vector2f perceived_center_circle_pos = test_pose_eigen_inv * center_circle_model.head<2>();
  perceived_center_circle.percept =
    Eigen::Vector3f{perceived_center_circle_pos.x(), perceived_center_circle_pos.y(), -test_pose.theta} + test_noise[1];
  perceived_center_circle.model = center_circle_model;
  perceived_center_circle.cov_percept = cov_percept;

  std::vector<nomadz_modeling::PerceivedFeaturePose> perceived_feature_poses;
  perceived_feature_poses.push_back(perceived_penalty_mark);
  perceived_feature_poses.push_back(perceived_center_circle);

  nomadz_modeling::PoseRegistrator pose_registrator;

  std::vector<nomadz_modeling::RegisteredRobotPose> registered_robot_poses;

  pose_registrator.registerPoses(test_pose, perceived_feature_poses, registered_robot_poses);

  EXPECT_TRUE(registered_robot_poses.size() == 2);
}

TEST(testPoseRegistration, InexactFeaturePosesInexactFieldPose) {

  const nomadz_core::Pose2D test_pose_gt{1.0, 0.0, 0.0};
  const nomadz_core::Pose2D test_pose{1.1, 0.0, 0.0};
  const Eigen::Affine2f test_pose_eigen_gt = nomadz_core::toAffine2(test_pose_gt);
  const Eigen::Affine2f test_pose_eigen_inv_gt = test_pose_eigen_gt.inverse();

  std::array<Eigen::Vector3f, 2> test_noise = {Eigen::Vector3f{-0.00454064, 0.04921433, 0.05975387},
                                               Eigen::Vector3f{0.01455327, 0.0328405, 0.03236976}};

  Eigen::Matrix3f cov_percept = computeCov<2>(test_noise);

  const nomadz_configuration::Dims dims;

  const Eigen::Vector3f penalty_mark_model = Eigen::Vector3f{dims.x_pos_opponent_penalty_area, 0.F, 0.F};
  const Eigen::Vector3f center_circle_model = Eigen::Vector3f{0.F, 0.F, 0.F};

  nomadz_modeling::PerceivedFeaturePose perceived_penalty_mark;
  Eigen::Vector2f perceived_penalty_mark_pos = test_pose_eigen_inv_gt * penalty_mark_model.head<2>();
  perceived_penalty_mark.percept =
    Eigen::Vector3f{perceived_penalty_mark_pos.x(), perceived_penalty_mark_pos.y(), -test_pose_gt.theta} + test_noise[0];
  perceived_penalty_mark.model = penalty_mark_model;
  perceived_penalty_mark.cov_percept = cov_percept;

  nomadz_modeling::PerceivedFeaturePose perceived_center_circle;
  Eigen::Vector2f perceived_center_circle_pos = test_pose_eigen_inv_gt * center_circle_model.head<2>();
  perceived_center_circle.percept =
    Eigen::Vector3f{perceived_center_circle_pos.x(), perceived_center_circle_pos.y(), -test_pose_gt.theta} + test_noise[1];
  perceived_center_circle.model = center_circle_model;
  perceived_center_circle.cov_percept = cov_percept;

  std::vector<nomadz_modeling::PerceivedFeaturePose> perceived_feature_poses;
  perceived_feature_poses.push_back(perceived_penalty_mark);
  perceived_feature_poses.push_back(perceived_center_circle);

  nomadz_modeling::PoseRegistrator pose_registrator;

  std::vector<nomadz_modeling::RegisteredRobotPose> registered_robot_poses;

  pose_registrator.registerPoses(test_pose, perceived_feature_poses, registered_robot_poses);

  EXPECT_TRUE(registered_robot_poses.size() == 2);
}
