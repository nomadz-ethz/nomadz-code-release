#include <gtest/gtest.h>

#include <Eigen/Core>
#include <iostream>

#include "nomadz_configuration/field_dimensions.hpp"
#include "nomadz_core/geometry/pose.hpp"
#include "nomadz_modeling/line_registrator.hpp"

nomadz_configuration::Line transformLine(Eigen::Affine2f pose,
                                         nomadz_configuration::Line line,
                                         Eigen::Vector2f noise_begin = Eigen::Vector2f::Zero(),
                                         Eigen::Vector2f noise_end = Eigen::Vector2f::Zero()) {
  return {pose * line.from + noise_begin, pose * line.to + noise_end};
}

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

TEST(testLineRegistration, ExactLinesInitialFieldPose) {
  const nomadz_core::Pose2D test_pose{0.0, 0.0, 0.0};
  const Eigen::Affine2f test_pose_eigen = nomadz_core::toAffine2(test_pose);
  const Eigen::Affine2f test_pose_eigen_inv = test_pose_eigen.inverse();

  const nomadz_configuration::FieldLines field_lines_gt;
  std::array<unsigned int, 5> test_indices = {0, 5, 10, 15, 16};

  std::vector<nomadz_modeling::PerceivedLine> perceived_lines;
  perceived_lines.reserve(test_indices.size());
  for (const unsigned int index : test_indices) {
    nomadz_configuration::Line line{transformLine(test_pose_eigen_inv, field_lines_gt.field_lines[index])};
    nomadz_modeling::PerceivedLine perceived_line{line, Eigen::Matrix2f::Zero()};
    perceived_lines.push_back(perceived_line);
  }
  nomadz_modeling::LineRegistrator line_registrator;

  std::vector<nomadz_modeling::RegisteredLine> registered_lines;

  line_registrator.registerLines(test_pose, perceived_lines, registered_lines);

  EXPECT_TRUE(registered_lines.size() == perceived_lines.size());
  for (unsigned int i = 0; i < test_indices.size(); ++i) {
    const unsigned int index = test_indices[i];
    const nomadz_configuration::Line gt_line = field_lines_gt.field_lines[index];
    const nomadz_modeling::RegisteredLine registered_line = registered_lines[i];

    EXPECT_TRUE(gt_line.to.isApprox(registered_line.model.to));
    EXPECT_TRUE(gt_line.to.isApprox(registered_line.percept.to));
    EXPECT_TRUE(gt_line.from.isApprox(registered_line.model.from));
    EXPECT_TRUE(gt_line.from.isApprox(registered_line.percept.from));
  }
}

TEST(testLineRegistration, ExactLinesDifferentFieldPose) {
  const nomadz_core::Pose2D test_pose{1.0, 0.5, 0.2};
  const Eigen::Affine2f test_pose_eigen = nomadz_core::toAffine2(test_pose);
  const Eigen::Affine2f test_pose_eigen_inv = test_pose_eigen.inverse();

  const nomadz_configuration::FieldLines field_lines_gt;
  std::array<unsigned int, 5> test_indices = {0, 5, 10, 15, 16};

  std::vector<nomadz_modeling::PerceivedLine> perceived_lines;
  perceived_lines.reserve(test_indices.size());
  for (const unsigned int index : test_indices) {
    nomadz_configuration::Line line{transformLine(test_pose_eigen_inv, field_lines_gt.field_lines[index])};
    nomadz_modeling::PerceivedLine perceived_line{line, Eigen::Matrix2f::Zero()};
    perceived_lines.push_back(perceived_line);
  }
  nomadz_modeling::LineRegistrator line_registrator;

  std::vector<nomadz_modeling::RegisteredLine> registered_lines;

  line_registrator.registerLines(test_pose, perceived_lines, registered_lines);

  EXPECT_TRUE(registered_lines.size() == perceived_lines.size());
  for (unsigned int i = 0; i < test_indices.size(); ++i) {
    const unsigned int index = test_indices[i];
    const nomadz_configuration::Line gt_line = field_lines_gt.field_lines[index];
    const nomadz_modeling::RegisteredLine registered_line = registered_lines[i];

    EXPECT_TRUE(gt_line.to.isApprox(registered_line.model.to));
    EXPECT_TRUE(gt_line.to.isApprox(test_pose_eigen * registered_line.percept.to));
    EXPECT_TRUE(gt_line.from.isApprox(registered_line.model.from));
    EXPECT_TRUE(gt_line.from.isApprox(test_pose_eigen * registered_line.percept.from));
  }
}

TEST(testLineRegistration, ExactLinesInexactPose) {
  const nomadz_core::Pose2D test_pose{1.0, 0.5, 0.2};
  const Eigen::Affine2f test_pose_eigen = nomadz_core::toAffine2(test_pose);
  const Eigen::Affine2f test_pose_eigen_inv = test_pose_eigen.inverse();

  const nomadz_configuration::FieldLines field_lines_gt;
  std::array<unsigned int, 5> test_indices = {0, 5, 10, 15, 16};

  std::vector<nomadz_modeling::PerceivedLine> perceived_lines;
  perceived_lines.reserve(test_indices.size());
  for (const unsigned int index : test_indices) {
    nomadz_configuration::Line line{transformLine(test_pose_eigen_inv, field_lines_gt.field_lines[index])};
    nomadz_modeling::PerceivedLine perceived_line{line, Eigen::Matrix2f::Zero()};
    perceived_lines.push_back(perceived_line);
  }
  nomadz_modeling::LineRegistrator line_registrator;

  std::vector<nomadz_modeling::RegisteredLine> registered_lines;

  line_registrator.registerLines({1.05, 0.45, 0.25}, perceived_lines, registered_lines);

  EXPECT_TRUE(registered_lines.size() == perceived_lines.size());
  for (unsigned int i = 0; i < test_indices.size(); ++i) {
    const unsigned int index = test_indices[i];
    const nomadz_configuration::Line gt_line = field_lines_gt.field_lines[index];
    const nomadz_modeling::RegisteredLine registered_line = registered_lines[i];

    EXPECT_TRUE(gt_line.to.isApprox(registered_line.model.to));
    EXPECT_TRUE(gt_line.to.isApprox(test_pose_eigen * registered_line.percept.to));
    EXPECT_TRUE(gt_line.from.isApprox(registered_line.model.from));
    EXPECT_TRUE(gt_line.from.isApprox(test_pose_eigen * registered_line.percept.from));
  }
}

TEST(testLineRegistration, InexactLinesExactPose) {

  const nomadz_core::Pose2D test_pose{0.0, 0.0, 0.0};
  const Eigen::Affine2f test_pose_eigen = nomadz_core::toAffine2(test_pose);
  const Eigen::Affine2f test_pose_eigen_inv = test_pose_eigen.inverse();

  const nomadz_configuration::FieldLines field_lines_gt;
  std::array<unsigned int, 5> test_indices = {0, 5, 10, 15, 16};
  std::array<Eigen::Vector2f, 10> test_noise = {Eigen::Vector2f(-0.00454064, 0.04921433),
                                                Eigen::Vector2f(0.01455327, 0.0328405),
                                                Eigen::Vector2f(-0.00493619, -0.07921715),
                                                Eigen::Vector2f(0.04452836, 0.01597251),
                                                Eigen::Vector2f(-0.06971589, -0.04068868),
                                                Eigen::Vector2f(0.03602708, -0.07199052),
                                                Eigen::Vector2f(-0.04257796, 0.04478105),
                                                Eigen::Vector2f(0.08552297, 0.09136167),
                                                Eigen::Vector2f(-0.05536088, 0.05201618),
                                                Eigen::Vector2f(0.03852832, 0.058947)};

  Eigen::Matrix2f cov_noise = computeCov<10>(test_noise);
  std::vector<nomadz_modeling::PerceivedLine> perceived_lines;
  perceived_lines.reserve(test_indices.size());
  for (unsigned int i = 0; i < test_indices.size(); ++i) {
    unsigned int index = test_indices[i];
    nomadz_configuration::Line line{
      transformLine(test_pose_eigen_inv, field_lines_gt.field_lines[index], test_noise[i], test_noise[5 + i])};
    nomadz_modeling::PerceivedLine perceived_line{line, cov_noise};
    perceived_lines.push_back(perceived_line);
  }
  nomadz_modeling::LineRegistrator line_registrator;

  std::vector<nomadz_modeling::RegisteredLine> registered_lines;

  line_registrator.registerLines(test_pose, perceived_lines, registered_lines);

  EXPECT_TRUE(registered_lines.size() == perceived_lines.size());
  for (unsigned int i = 0; i < test_indices.size(); ++i) {
    const unsigned int index = test_indices[i];
    const nomadz_configuration::Line gt_line = field_lines_gt.field_lines[index];
    const nomadz_modeling::RegisteredLine registered_line = registered_lines[i];

    EXPECT_TRUE(gt_line.to.isApprox(registered_line.model.to));
    EXPECT_TRUE(gt_line.from.isApprox(registered_line.model.from));
  }
}

TEST(testLineRegistration, InexactLinesInexactPose) {

  const nomadz_core::Pose2D test_pose_gt{1.1, 0.45, 0.3};
  const nomadz_core::Pose2D test_pose{1.05, 0.45, 0.25};
  const Eigen::Affine2f test_pose_eigen_gt = nomadz_core::toAffine2(test_pose_gt);
  const Eigen::Affine2f test_pose_eigen_inv_gt = test_pose_eigen_gt.inverse();

  const nomadz_configuration::FieldLines field_lines_gt;
  std::array<unsigned int, 5> test_indices = {0, 5, 10, 15, 16};
  std::array<Eigen::Vector2f, 10> test_noise = {Eigen::Vector2f(-0.00454064, 0.04921433),
                                                Eigen::Vector2f(0.01455327, 0.0328405),
                                                Eigen::Vector2f(-0.00493619, -0.07921715),
                                                Eigen::Vector2f(0.04452836, 0.01597251),
                                                Eigen::Vector2f(-0.06971589, -0.04068868),
                                                Eigen::Vector2f(0.03602708, -0.07199052),
                                                Eigen::Vector2f(-0.04257796, 0.04478105),
                                                Eigen::Vector2f(0.08552297, 0.09136167),
                                                Eigen::Vector2f(-0.05536088, 0.05201618),
                                                Eigen::Vector2f(0.03852832, 0.058947)};

  Eigen::Matrix2f cov_noise = computeCov<10>(test_noise);
  std::vector<nomadz_modeling::PerceivedLine> perceived_lines;
  perceived_lines.reserve(test_indices.size());
  for (unsigned int i = 0; i < test_indices.size(); ++i) {
    unsigned int index = test_indices[i];
    nomadz_configuration::Line line{
      transformLine(test_pose_eigen_inv_gt, field_lines_gt.field_lines[index], test_noise[i], test_noise[5 + i])};
    nomadz_modeling::PerceivedLine perceived_line{line, cov_noise};
    perceived_lines.push_back(perceived_line);
  }
  nomadz_modeling::LineRegistrator line_registrator;

  std::vector<nomadz_modeling::RegisteredLine> registered_lines;

  line_registrator.registerLines(test_pose, perceived_lines, registered_lines);

  EXPECT_TRUE(registered_lines.size() == perceived_lines.size());
  for (unsigned int i = 0; i < test_indices.size(); ++i) {
    const unsigned int index = test_indices[i];
    const nomadz_configuration::Line gt_line = field_lines_gt.field_lines[index];
    const nomadz_modeling::RegisteredLine registered_line = registered_lines[i];

    EXPECT_TRUE(gt_line.to.isApprox(registered_line.model.to));
    EXPECT_TRUE(gt_line.from.isApprox(registered_line.model.from));
  }
}
