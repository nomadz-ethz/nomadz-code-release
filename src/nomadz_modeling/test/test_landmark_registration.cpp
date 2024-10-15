#include <gtest/gtest.h>

#include <Eigen/Core>

#include "nomadz_configuration/field_dimensions.hpp"
#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_modeling/landmark_registrator.hpp"

template <int N> Eigen::Matrix2f computeCov(std::array<Eigen::Vector2f, N>& points) {
  // Compute the mean of the points
  Eigen::Vector2f mean = Eigen::Vector2f::Zero();
  for (const auto& point : points) {
    mean += point;
  }
  mean /= N;

  // Subtract the mean from each point to get mean-centered data
  Eigen::MatrixXf centered(2, N);
  for (int i = 0; i < N; ++i) {
    centered.col(i) = points[i] - mean;
  }

  // Compute the covariance matrix
  Eigen::Matrix2f covariance = (centered * centered.transpose()) / (N - 1);

  return covariance;
}

TEST(testLandmarkRegistration, ExactCircleAndPenaltyExactFieldPose) {

  const nomadz_core::Pose2D test_pose{1.0, 0.0, 0.0};
  const Eigen::Affine2f test_pose_eigen = nomadz_core::toAffine2(test_pose);
  const Eigen::Affine2f test_pose_eigen_inv = test_pose_eigen.inverse();

  const nomadz_configuration::Dims dims;

  const Eigen::Vector2f penalty_mark_model = Eigen::Vector2f{dims.x_pos_own_penalty_mark, 0.F};
  const Eigen::Vector2f center_circle_model = Eigen::Vector2f{0.F, 0.F};

  nomadz_modeling::PerceivedLandmark perceived_penalty_mark;
  perceived_penalty_mark.percept = test_pose_eigen_inv * penalty_mark_model;
  perceived_penalty_mark.cov_percept = Eigen::Matrix2f::Zero();
  perceived_penalty_mark.landmark_type = nomadz_modeling::LandmarkType::PENALTY_MARK;

  nomadz_modeling::PerceivedLandmark perceived_center_circle;
  perceived_center_circle.percept = test_pose_eigen_inv * center_circle_model;
  perceived_center_circle.cov_percept = Eigen::Matrix2f::Zero();
  perceived_center_circle.landmark_type = nomadz_modeling::LandmarkType::CIRCLE;

  std::vector<nomadz_modeling::PerceivedLandmark> perceived_landmarks;
  perceived_landmarks.push_back(perceived_penalty_mark);
  perceived_landmarks.push_back(perceived_center_circle);

  nomadz_modeling::LandmarkRegistrator landmark_registrator;

  std::vector<nomadz_modeling::RegisteredLandmark> registered_landmarks;

  landmark_registrator.registerLandmarks(test_pose, perceived_landmarks, registered_landmarks);

  EXPECT_TRUE(registered_landmarks.size() == 2);
  for (const nomadz_modeling::RegisteredLandmark& registered_landmark : registered_landmarks) {
    EXPECT_TRUE(registered_landmark.model.isApprox(test_pose_eigen * registered_landmark.percept));
  }
}

TEST(testLandmarkRegistration, ExactCircleAndPenaltyInexactFieldPose) {

  const nomadz_core::Pose2D test_pose_gt{1.0, 0.0, 0.0};
  const nomadz_core::Pose2D test_pose{0.0, 0.0, 0.0};
  const Eigen::Affine2f test_pose_eigen_gt = nomadz_core::toAffine2(test_pose_gt);
  const Eigen::Affine2f test_pose_eigen_inv_gt = test_pose_eigen_gt.inverse();

  const nomadz_configuration::Dims dims;

  const Eigen::Vector2f penalty_mark_model = Eigen::Vector2f{dims.x_pos_own_penalty_mark, 0.F};
  const Eigen::Vector2f center_circle_model = Eigen::Vector2f{0.F, 0.F};

  nomadz_modeling::PerceivedLandmark perceived_penalty_mark;
  perceived_penalty_mark.percept = test_pose_eigen_inv_gt * penalty_mark_model;
  perceived_penalty_mark.cov_percept = Eigen::Matrix2f::Zero();
  perceived_penalty_mark.landmark_type = nomadz_modeling::LandmarkType::PENALTY_MARK;

  nomadz_modeling::PerceivedLandmark perceived_center_circle;
  perceived_center_circle.percept = test_pose_eigen_inv_gt * center_circle_model;
  perceived_center_circle.cov_percept = Eigen::Matrix2f::Zero();
  perceived_center_circle.landmark_type = nomadz_modeling::LandmarkType::CIRCLE;

  std::vector<nomadz_modeling::PerceivedLandmark> perceived_landmarks;
  perceived_landmarks.push_back(perceived_penalty_mark);
  perceived_landmarks.push_back(perceived_center_circle);

  nomadz_modeling::LandmarkRegistrator landmark_registrator;

  std::vector<nomadz_modeling::RegisteredLandmark> registered_landmarks;

  landmark_registrator.registerLandmarks(test_pose, perceived_landmarks, registered_landmarks);

  EXPECT_TRUE(registered_landmarks.size() == 2);
  for (const nomadz_modeling::RegisteredLandmark& registered_landmark : registered_landmarks) {
    EXPECT_TRUE(registered_landmark.model.isApprox(test_pose_eigen_gt * registered_landmark.percept));
  }
}

TEST(testLandmarkRegistration, InexactCircleAndPenaltyExactFieldPose) {

  const nomadz_core::Pose2D test_pose{1.0, 0.0, 0.0};
  const Eigen::Affine2f test_pose_eigen = nomadz_core::toAffine2(test_pose);
  const Eigen::Affine2f test_pose_eigen_inv = test_pose_eigen.inverse();

  std::array<Eigen::Vector2f, 2> test_noise = {Eigen::Vector2f{-0.00454064, 0.04921433},
                                               Eigen::Vector2f{0.01455327, 0.0328405}};

  Eigen::Matrix2f cov_percept = computeCov<2>(test_noise);

  const nomadz_configuration::Dims dims;

  const Eigen::Vector2f penalty_mark_model = Eigen::Vector2f{dims.x_pos_own_penalty_mark, 0.F};
  const Eigen::Vector2f center_circle_model = Eigen::Vector2f{0.F, 0.F};

  nomadz_modeling::PerceivedLandmark perceived_penalty_mark;
  perceived_penalty_mark.percept = test_pose_eigen_inv * penalty_mark_model + test_noise[0];
  perceived_penalty_mark.cov_percept = cov_percept;
  perceived_penalty_mark.landmark_type = nomadz_modeling::LandmarkType::PENALTY_MARK;

  nomadz_modeling::PerceivedLandmark perceived_center_circle;
  perceived_center_circle.percept = test_pose_eigen_inv * center_circle_model + test_noise[1];
  perceived_center_circle.cov_percept = cov_percept;
  perceived_center_circle.landmark_type = nomadz_modeling::LandmarkType::CIRCLE;

  std::vector<nomadz_modeling::PerceivedLandmark> perceived_landmarks;
  perceived_landmarks.push_back(perceived_penalty_mark);
  perceived_landmarks.push_back(perceived_center_circle);

  nomadz_modeling::LandmarkRegistrator landmark_registrator;

  std::vector<nomadz_modeling::RegisteredLandmark> registered_landmarks;

  landmark_registrator.registerLandmarks(test_pose, perceived_landmarks, registered_landmarks);

  EXPECT_TRUE(registered_landmarks.size() == 2);
  if (registered_landmarks.size() == 2) {
    EXPECT_TRUE(registered_landmarks[0].model.isApprox(penalty_mark_model));
    EXPECT_TRUE(registered_landmarks[1].model.isApprox(center_circle_model));
  }
}

TEST(testLandmarkRegistration, InexactCircleAndPenaltyInexactFieldPose) {

  const nomadz_core::Pose2D test_pose_gt{1.0, 0.0, 0.0};
  const nomadz_core::Pose2D test_pose{0.0, 0.0, 0.0};
  const Eigen::Affine2f test_pose_eigen_gt = nomadz_core::toAffine2(test_pose_gt);
  const Eigen::Affine2f test_pose_eigen_inv_gt = test_pose_eigen_gt.inverse();

  std::array<Eigen::Vector2f, 2> test_noise = {Eigen::Vector2f{-0.00454064, 0.04921433},
                                               Eigen::Vector2f{0.01455327, 0.0328405}};

  Eigen::Matrix2f cov_percept = computeCov<2>(test_noise);

  const nomadz_configuration::Dims dims;

  const Eigen::Vector2f penalty_mark_model = Eigen::Vector2f{dims.x_pos_own_penalty_mark, 0.F};
  const Eigen::Vector2f center_circle_model = Eigen::Vector2f{0.F, 0.F};

  nomadz_modeling::PerceivedLandmark perceived_penalty_mark;
  perceived_penalty_mark.percept = test_pose_eigen_inv_gt * penalty_mark_model + test_noise[0];
  perceived_penalty_mark.cov_percept = cov_percept;
  perceived_penalty_mark.landmark_type = nomadz_modeling::LandmarkType::PENALTY_MARK;

  nomadz_modeling::PerceivedLandmark perceived_center_circle;
  perceived_center_circle.percept = test_pose_eigen_inv_gt * center_circle_model + test_noise[1];
  perceived_center_circle.cov_percept = cov_percept;
  perceived_center_circle.landmark_type = nomadz_modeling::LandmarkType::CIRCLE;

  std::vector<nomadz_modeling::PerceivedLandmark> perceived_landmarks;
  perceived_landmarks.push_back(perceived_penalty_mark);
  perceived_landmarks.push_back(perceived_center_circle);

  nomadz_modeling::LandmarkRegistrator landmark_registrator;

  std::vector<nomadz_modeling::RegisteredLandmark> registered_landmarks;

  landmark_registrator.registerLandmarks(test_pose, perceived_landmarks, registered_landmarks);

  EXPECT_TRUE(registered_landmarks.size() == 2);
  if (registered_landmarks.size() == 2) {
    EXPECT_TRUE(registered_landmarks[0].model.isApprox(penalty_mark_model));
    EXPECT_TRUE(registered_landmarks[1].model.isApprox(center_circle_model));
  }
}

TEST(testLandmarkRegistration, ExactIntersectionsExactFieldPose) {

  const nomadz_core::Pose2D test_pose{1.0, 0.0, 0.0};
  const Eigen::Affine2f test_pose_eigen = nomadz_core::toAffine2(test_pose);
  const Eigen::Affine2f test_pose_eigen_inv = test_pose_eigen.inverse();

  const nomadz_configuration::Corners corners;

  const std::vector<std::vector<Eigen::Vector2f>> possible_corners{corners.X_CORNER,
                                                                   corners.T_CORNER_0,
                                                                   corners.T_CORNER_90,
                                                                   corners.T_CORNER_180,
                                                                   corners.T_CORNER_270,
                                                                   corners.L_CORNER_0,
                                                                   corners.L_CORNER_90,
                                                                   corners.L_CORNER_180,
                                                                   corners.L_CORNER_270};

  const Eigen::Vector2f dir_0{1.0, 0.0};
  const Eigen::Vector2f dir_90{0.0, 1.0};
  const Eigen::Vector2f dir_180{-1.0, 0.0};
  const Eigen::Vector2f dir_270{0.0, -1.0};

  std::array<unsigned int, 9> test_indices = {0, 1, 2, 3, 4, 5, 6, 7, 8};
  for (const unsigned int i : test_indices) {
    std::vector<nomadz_modeling::PerceivedIntersection> perceived_intersections;
    for (const Eigen::Vector2f& intersection : possible_corners[i]) {
      nomadz_modeling::PerceivedIntersection perceived_intersection;
      perceived_intersection.percept = test_pose_eigen_inv * intersection;
      perceived_intersection.cov_percept = Eigen::Matrix2f::Zero();
      switch (i) {
      case 0:
      case 1:
      case 5:
        perceived_intersection.dir = test_pose_eigen_inv.rotation() * dir_0;
        break;
      case 2:
      case 6:
        perceived_intersection.dir = test_pose_eigen_inv.rotation() * dir_90;
        break;
      case 3:
      case 7:
        perceived_intersection.dir = test_pose_eigen_inv.rotation() * dir_180;
        break;
      case 4:
      default:
        perceived_intersection.dir = test_pose_eigen_inv.rotation() * dir_270;
        break;
      }

      if (i == 0) {
        perceived_intersection.intersection_type = nomadz_modeling::IntersectionType::X;
      } else if (i < 5) {
        perceived_intersection.intersection_type = nomadz_modeling::IntersectionType::T;
      } else {
        perceived_intersection.intersection_type = nomadz_modeling::IntersectionType::L;
      }

      perceived_intersections.push_back(perceived_intersection);
    }

    std::vector<nomadz_modeling::RegisteredLandmark> registered_intersections;
    nomadz_modeling::LandmarkRegistrator landmark_registrator;
    landmark_registrator.registerLandmarks(test_pose, perceived_intersections, registered_intersections);
    EXPECT_TRUE(perceived_intersections.size() == registered_intersections.size());
    for (int j = 0; j < static_cast<int>(registered_intersections.size()); ++j) {
      EXPECT_TRUE(possible_corners[i][j].isApprox(test_pose_eigen * registered_intersections[j].percept));
      EXPECT_TRUE(possible_corners[i][j].isApprox(registered_intersections[j].model));
    }
  }
}

TEST(testLandmarkRegistration, ExactIntersectionsInexactFieldPose) {

  const nomadz_core::Pose2D test_pose_gt{0.45, 0.0, 0.0};
  const nomadz_core::Pose2D test_pose{0.0, 0.0, 0.0};
  const Eigen::Affine2f test_pose_eigen_gt = nomadz_core::toAffine2(test_pose_gt);
  const Eigen::Affine2f test_pose_eigen_inv_gt = test_pose_eigen_gt.inverse();

  const nomadz_configuration::Corners corners;

  const std::vector<std::vector<Eigen::Vector2f>> possible_corners{corners.X_CORNER,
                                                                   corners.T_CORNER_0,
                                                                   corners.T_CORNER_90,
                                                                   corners.T_CORNER_180,
                                                                   corners.T_CORNER_270,
                                                                   corners.L_CORNER_0,
                                                                   corners.L_CORNER_90,
                                                                   corners.L_CORNER_180,
                                                                   corners.L_CORNER_270};

  const Eigen::Vector2f dir_0{1.0, 0.0};
  const Eigen::Vector2f dir_90{0.0, 1.0};
  const Eigen::Vector2f dir_180{-1.0, 0.0};
  const Eigen::Vector2f dir_270{0.0, -1.0};

  std::array<unsigned int, 9> test_indices = {0, 1, 2, 3, 4, 5, 6, 7, 8};
  for (const unsigned int i : test_indices) {
    std::vector<nomadz_modeling::PerceivedIntersection> perceived_intersections;
    for (const Eigen::Vector2f& intersection : possible_corners[i]) {
      nomadz_modeling::PerceivedIntersection perceived_intersection;
      perceived_intersection.percept = test_pose_eigen_inv_gt * intersection;
      perceived_intersection.cov_percept = Eigen::Matrix2f::Zero();
      switch (i) {
      case 0:
      case 1:
      case 5:
        perceived_intersection.dir = test_pose_eigen_inv_gt.rotation() * dir_0;
        break;
      case 2:
      case 6:
        perceived_intersection.dir = test_pose_eigen_inv_gt.rotation() * dir_90;
        break;
      case 3:
      case 7:
        perceived_intersection.dir = test_pose_eigen_inv_gt.rotation() * dir_180;
        break;
      case 4:
      default:
        perceived_intersection.dir = test_pose_eigen_inv_gt.rotation() * dir_270;
        break;
      }

      if (i == 0) {
        perceived_intersection.intersection_type = nomadz_modeling::IntersectionType::X;
      } else if (i < 5) {
        perceived_intersection.intersection_type = nomadz_modeling::IntersectionType::T;
      } else {
        perceived_intersection.intersection_type = nomadz_modeling::IntersectionType::L;
      }

      perceived_intersections.push_back(perceived_intersection);
    }

    std::vector<nomadz_modeling::RegisteredLandmark> registered_intersections;
    nomadz_modeling::LandmarkRegistrator landmark_registrator;
    landmark_registrator.registerLandmarks(test_pose, perceived_intersections, registered_intersections);
    EXPECT_TRUE(perceived_intersections.size() == registered_intersections.size());
    for (int j = 0; j < static_cast<int>(registered_intersections.size()); ++j) {
      EXPECT_TRUE(possible_corners[i][j].isApprox(test_pose_eigen_gt * registered_intersections[j].percept));
      EXPECT_TRUE(possible_corners[i][j].isApprox(registered_intersections[j].model));
    }
  }
}

TEST(testLandmarkRegistration, InexactIntersectionsExactFieldPose) {

  const nomadz_core::Pose2D test_pose{0.0, 0.0, 0.0};
  const Eigen::Affine2f test_pose_eigen = nomadz_core::toAffine2(test_pose);
  const Eigen::Affine2f test_pose_eigen_inv = test_pose_eigen.inverse();

  const nomadz_configuration::Corners corners;

  const std::vector<std::vector<Eigen::Vector2f>> possible_corners{corners.X_CORNER,
                                                                   corners.T_CORNER_0,
                                                                   corners.T_CORNER_90,
                                                                   corners.T_CORNER_180,
                                                                   corners.T_CORNER_270,
                                                                   corners.L_CORNER_0,
                                                                   corners.L_CORNER_90,
                                                                   corners.L_CORNER_180,
                                                                   corners.L_CORNER_270};

  const Eigen::Vector2f dir_0{1.0, 0.0};
  const Eigen::Vector2f dir_90{0.0, 1.0};
  const Eigen::Vector2f dir_180{-1.0, 0.0};
  const Eigen::Vector2f dir_270{0.0, -1.0};

  Eigen::Vector2f noise{-0.00454064, 0.04921433};
  std::array<unsigned int, 9> test_indices = {0, 1, 2, 3, 4, 5, 6, 7, 8};
  for (const unsigned int i : test_indices) {
    std::vector<nomadz_modeling::PerceivedIntersection> perceived_intersections;
    for (const Eigen::Vector2f& intersection : possible_corners[i]) {
      nomadz_modeling::PerceivedIntersection perceived_intersection;
      perceived_intersection.percept = test_pose_eigen_inv * intersection + noise;
      perceived_intersection.cov_percept = Eigen::Matrix2f::Zero();
      switch (i) {
      case 0:
      case 1:
      case 5:
        perceived_intersection.dir = test_pose_eigen_inv.rotation() * dir_0;
        break;
      case 2:
      case 6:
        perceived_intersection.dir = test_pose_eigen_inv.rotation() * dir_90;
        break;
      case 3:
      case 7:
        perceived_intersection.dir = test_pose_eigen_inv.rotation() * dir_180;
        break;
      case 4:
      default:
        perceived_intersection.dir = test_pose_eigen_inv.rotation() * dir_270;
        break;
      }

      if (i == 0) {
        perceived_intersection.intersection_type = nomadz_modeling::IntersectionType::X;
      } else if (i < 5) {
        perceived_intersection.intersection_type = nomadz_modeling::IntersectionType::T;
      } else {
        perceived_intersection.intersection_type = nomadz_modeling::IntersectionType::L;
      }

      perceived_intersections.push_back(perceived_intersection);
    }

    std::vector<nomadz_modeling::RegisteredLandmark> registered_intersections;
    nomadz_modeling::LandmarkRegistrator landmark_registrator;
    landmark_registrator.registerLandmarks(test_pose, perceived_intersections, registered_intersections);
    EXPECT_TRUE(perceived_intersections.size() == registered_intersections.size());
    for (int j = 0; j < static_cast<int>(registered_intersections.size()); ++j) {
      EXPECT_TRUE(possible_corners[i][j].isApprox(registered_intersections[j].model));
    }
  }
}

TEST(testLandmarkRegistration, InexactIntersectionsInexactFieldPose) {

  const nomadz_core::Pose2D test_pose_gt{0.45, 0.0, 0.0};
  const nomadz_core::Pose2D test_pose{0.0, 0.0, 0.0};
  const Eigen::Affine2f test_pose_eigen_gt = nomadz_core::toAffine2(test_pose_gt);
  const Eigen::Affine2f test_pose_eigen_inv_gt = test_pose_eigen_gt.inverse();

  const nomadz_configuration::Corners corners;

  const std::vector<std::vector<Eigen::Vector2f>> possible_corners{corners.X_CORNER,
                                                                   corners.T_CORNER_0,
                                                                   corners.T_CORNER_90,
                                                                   corners.T_CORNER_180,
                                                                   corners.T_CORNER_270,
                                                                   corners.L_CORNER_0,
                                                                   corners.L_CORNER_90,
                                                                   corners.L_CORNER_180,
                                                                   corners.L_CORNER_270};

  const Eigen::Vector2f dir_0{1.0, 0.0};
  const Eigen::Vector2f dir_90{0.0, 1.0};
  const Eigen::Vector2f dir_180{-1.0, 0.0};
  const Eigen::Vector2f dir_270{0.0, -1.0};

  Eigen::Vector2f noise{-0.00454064, 0.04921433};
  std::array<unsigned int, 9> test_indices = {0, 1, 2, 3, 4, 5, 6, 7, 8};
  for (const unsigned int i : test_indices) {
    std::vector<nomadz_modeling::PerceivedIntersection> perceived_intersections;
    for (const Eigen::Vector2f& intersection : possible_corners[i]) {
      nomadz_modeling::PerceivedIntersection perceived_intersection;
      perceived_intersection.percept = test_pose_eigen_inv_gt * intersection + noise;
      perceived_intersection.cov_percept = Eigen::Matrix2f::Zero();
      switch (i) {
      case 0:
      case 1:
      case 5:
        perceived_intersection.dir = test_pose_eigen_inv_gt.rotation() * dir_0;
        break;
      case 2:
      case 6:
        perceived_intersection.dir = test_pose_eigen_inv_gt.rotation() * dir_90;
        break;
      case 3:
      case 7:
        perceived_intersection.dir = test_pose_eigen_inv_gt.rotation() * dir_180;
        break;
      case 4:
      default:
        perceived_intersection.dir = test_pose_eigen_inv_gt.rotation() * dir_270;
        break;
      }

      if (i == 0) {
        perceived_intersection.intersection_type = nomadz_modeling::IntersectionType::X;
      } else if (i < 5) {
        perceived_intersection.intersection_type = nomadz_modeling::IntersectionType::T;
      } else {
        perceived_intersection.intersection_type = nomadz_modeling::IntersectionType::L;
      }

      perceived_intersections.push_back(perceived_intersection);
    }

    std::vector<nomadz_modeling::RegisteredLandmark> registered_intersections;
    nomadz_modeling::LandmarkRegistrator landmark_registrator;
    landmark_registrator.registerLandmarks(test_pose, perceived_intersections, registered_intersections);
    EXPECT_TRUE(perceived_intersections.size() == registered_intersections.size());
    for (int j = 0; j < static_cast<int>(registered_intersections.size()); ++j) {
      EXPECT_TRUE(possible_corners[i][j].isApprox(registered_intersections[j].model));
    }
  }
}
