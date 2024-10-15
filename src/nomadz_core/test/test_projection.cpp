#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "nomadz_core/geometry/projection.hpp"

using Eigen::Affine2f;
using Eigen::Affine3f;
using nomadz_core::expandTo3D;
using nomadz_core::Pose;
using nomadz_core::Pose2D;
using nomadz_core::projectTo2D;

TEST(ProjectionTest, AffineOnlyProjection) {
  // Test case 1: Project a point (1, 2, 3) to 2D
  Affine3f pose1 = Affine3f::Identity();
  Affine3f pose2 = Affine3f::Identity() * Eigen::Translation3f(1.F, 2.F, 3.F);
  Affine2f pose2_2d = Affine2f::Identity() * Eigen::Translation2f(1.F, 2.F);
  Affine3f pose3 = Eigen::Translation3f(1.F, 2.F, 3.F) * Eigen::AngleAxisf(1.F, Eigen::Vector3f::UnitZ());
  Affine2f pose3_2d = Eigen::Translation2f(1.F, 2.F) * Eigen::Rotation2Df(1.F);

  EXPECT_TRUE(projectTo2D(expandTo3D(Affine2f::Identity())).matrix().isApprox(Affine2f::Identity().matrix()));
  EXPECT_TRUE(projectTo2D(expandTo3D(pose2_2d)).matrix().isApprox(pose2_2d.matrix()));
  EXPECT_TRUE(projectTo2D(expandTo3D(pose3_2d)).matrix().isApprox(pose3_2d.matrix()));

  EXPECT_TRUE(projectTo2D(pose1).matrix().isApprox(Affine2f::Identity().matrix()));
  EXPECT_TRUE(projectTo2D(pose2).matrix().isApprox(pose2_2d.matrix()));
  EXPECT_TRUE(projectTo2D(pose3).matrix().isApprox(pose3_2d.matrix()));
}

TEST(ProjectionTest, AffineBoth) {
  // Test case 1: Project a point (1, 2, 3) to 2D
  Affine3f pose1 = Affine3f::Identity();
  Affine3f pose2 = Affine3f::Identity() * Eigen::Translation3f(1.F, 2.F, 3.F);
  Affine3f pose3 = Eigen::Translation3f(1.F, 2.F, 3.F) * Eigen::AngleAxisf(1.F, Eigen::Vector3f::UnitZ());
  EXPECT_TRUE(expandTo3D(projectTo2D(pose1)).matrix().isApprox(pose1.matrix()));
  EXPECT_TRUE(expandTo3D(projectTo2D(pose2), 3.F).matrix().isApprox(pose2.matrix()));
  EXPECT_TRUE(expandTo3D(projectTo2D(pose3), 3.F).matrix().isApprox(pose3.matrix()));
}

TEST(ProjectionTest, PoseProjection) {
  // Test case 1: Project a point (1, 2, 3) to 2D
  Pose pose1{};
  pose1.translation << 0.F, 0.F, 0.F;
  pose1.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(0.F, Eigen::Vector3f::UnitZ()));
  Pose pose2{};
  pose2.translation << 1.F, 2.F, 3.F;
  pose2.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(0.F, Eigen::Vector3f::UnitZ()));
  Pose2D pose2_2d{1.F, 2.F, 0.F};
  Pose pose3{};
  pose3.translation << 1.F, 2.F, 3.F;
  pose3.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(1.F, Eigen::Vector3f::UnitZ()));
  Pose2D pose3_2d{1.F, 2.F, 1.F};

  EXPECT_TRUE(projectTo2D(expandTo3D(Pose2D{})) == Pose2D{});
  EXPECT_TRUE(projectTo2D(expandTo3D(pose2_2d)) == pose2_2d);
  EXPECT_TRUE(projectTo2D(expandTo3D(pose3_2d)) == pose3_2d);

  EXPECT_TRUE(projectTo2D(pose1) == Pose2D{});
  EXPECT_TRUE(projectTo2D(pose2) == pose2_2d);
  EXPECT_TRUE(projectTo2D(pose3) == pose3_2d);
}

TEST(ProjectionTest, PoseBoth) {
  // Test case 1: Project a point (1, 2, 3) to 2D
  Pose pose1{};
  pose1.translation << 0.F, 0.F, 0.F;
  pose1.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(0.F, Eigen::Vector3f::UnitZ()));
  Pose pose2{};
  pose2.translation << 1.F, 2.F, 3.F;
  pose2.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(0.F, Eigen::Vector3f::UnitZ()));
  Pose pose3{};
  pose3.translation << 1.F, 2.F, 3.F;
  pose3.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(1.F, Eigen::Vector3f::UnitZ()));

  EXPECT_TRUE(expandTo3D(projectTo2D(pose1)).translation == pose1.translation);
  EXPECT_TRUE(expandTo3D(projectTo2D(pose1)).orientation.coeffs() == pose1.orientation.coeffs());

  EXPECT_TRUE(expandTo3D(projectTo2D(pose2), 3.F).translation == pose2.translation);
  EXPECT_TRUE(expandTo3D(projectTo2D(pose2), 3.F).orientation.coeffs() == pose2.orientation.coeffs());

  EXPECT_TRUE(expandTo3D(projectTo2D(pose3), 3.F).translation == pose3.translation);
  EXPECT_TRUE(expandTo3D(projectTo2D(pose3), 3.F).orientation.coeffs() == pose3.orientation.coeffs());
}
