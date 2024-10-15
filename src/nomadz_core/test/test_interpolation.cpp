#include <gtest/gtest.h>

#include <array>
#include <Eigen/Core>

#include "nomadz_core/geometry/interpolation.hpp"

TEST(PolynomTest, BezierInterpolation) {
  // Test Bezier interpolation
  Eigen::Vector3d p0(0, 0, 0);
  Eigen::Vector3d p1(1, 2, 3);
  Eigen::Vector3d p2(4, 5, 6);
  Eigen::Vector3d p3(7, 8, 9);

  std::array<Eigen::Vector3d, 4> control_points = {p0, p1, p2, p3};
  auto interpolated_point = nomadz_core::interpolation::bezierInterpolation<Eigen::Vector3d, 3>(control_points, 0.);

  // Check the interpolated point
  EXPECT_DOUBLE_EQ(interpolated_point.x(), 0.);
  EXPECT_DOUBLE_EQ(interpolated_point.y(), 0.);
  EXPECT_DOUBLE_EQ(interpolated_point.z(), 0.);

  interpolated_point = nomadz_core::interpolation::bezierInterpolation<Eigen::Vector3d, 3>(control_points, 1.);

  // Check the interpolated point
  EXPECT_DOUBLE_EQ(interpolated_point.x(), 7.);
  EXPECT_DOUBLE_EQ(interpolated_point.y(), 8.);
  EXPECT_DOUBLE_EQ(interpolated_point.z(), 9.);
}

TEST(PolynomTest, RationalBezierInterpolation) {
  // Test Rational Bezier interpolation
  Eigen::Vector3d p0(0, 0, 0);
  Eigen::Vector3d p1(1, 2, 3);
  Eigen::Vector3d p2(4, 5, 6);
  Eigen::Vector3d p3(7, 8, 9);
  std::array<Eigen::Vector3d, 4> control_points = {p0, p1, p2, p3};
  std::array<float, 4> weights = {1.0, 1.0, 1.0, 1.0};
  auto interpolated_point =
    nomadz_core::interpolation::rationalBezierInterpolation<Eigen::Vector3d, 3>(control_points, weights, 0.);
  // Check the interpolated point
  EXPECT_DOUBLE_EQ(interpolated_point.x(), 0.);
  EXPECT_DOUBLE_EQ(interpolated_point.y(), 0.);
  EXPECT_DOUBLE_EQ(interpolated_point.z(), 0.);
  interpolated_point =
    nomadz_core::interpolation::rationalBezierInterpolation<Eigen::Vector3d, 3>(control_points, weights, 1.);
  // Check the interpolated point
  EXPECT_DOUBLE_EQ(interpolated_point.x(), 7.);
  EXPECT_DOUBLE_EQ(interpolated_point.y(), 8.);
  EXPECT_DOUBLE_EQ(interpolated_point.z(), 9.);
}
