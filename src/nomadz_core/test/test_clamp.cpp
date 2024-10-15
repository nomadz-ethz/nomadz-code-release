#include <gtest/gtest.h>

#include "nomadz_core/math/clamp.hpp"

using Eigen::Vector2f;
using Eigen::Vector3f;
using nomadz_core::ellipsoidClamp;
using nomadz_core::rectClamp;

TEST(ElllipsoidClamp, Vector3) {
  Vector3f current_vec1(2.F, 0.F, 0.F);
  Vector3f current_vec2(0.F, 2.F, 0.F);
  Vector3f current_vec3(0.F, 0.F, 2.F);
  Vector3f current_vec4(2.F, 2.F, 2.F);
  Vector3f ref_vec(1.F, 1.F, 1.F);

  // Test ellipsoidClamp function
  EXPECT_EQ(ellipsoidClamp(current_vec1, ref_vec), Vector3f(1.F, 0.F, 0.F));
  EXPECT_EQ(ellipsoidClamp(current_vec2, ref_vec), Vector3f(0.F, 1.F, 0.F));
  EXPECT_EQ(ellipsoidClamp(current_vec3, ref_vec), Vector3f(0.F, 0.F, 1.F));
  EXPECT_EQ(ellipsoidClamp(current_vec4, ref_vec), Vector3f(1.F, 1.F, 1.F) * std::sqrt(3.F) / 3.F);
}

TEST(RectClamp, Vector3) {
  Vector3f current_vec1(2.F, 0.F, 0.F);
  Vector3f current_vec2(0.F, 2.F, 0.F);
  Vector3f current_vec3(0.F, 0.F, 2.F);
  Vector3f ref_vec(1.F, 1.F, 1.F);

  // Test rectClamp function
  EXPECT_EQ(rectClamp(current_vec1, ref_vec), Vector3f(1.F, 0.F, 0.F));
  EXPECT_EQ(rectClamp(current_vec2, ref_vec), Vector3f(0.F, 1.F, 0.F));
  EXPECT_EQ(rectClamp(current_vec3, ref_vec), Vector3f(0.F, 0.F, 1.F));
}

TEST(EllipsoidClamp, Vector2) {
  Vector2f current_vec1(2.F, 0.F);
  Vector2f current_vec2(0.F, 2.F);
  Vector2f current_vec3(2.F, 2.F);
  Vector2f ref_vec(1.F, 1.F);

  // Test ellipsoidClamp function
  EXPECT_EQ(ellipsoidClamp(current_vec1, ref_vec), Vector2f(1.F, 0.F));
  EXPECT_EQ(ellipsoidClamp(current_vec2, ref_vec), Vector2f(0.F, 1.F));
  EXPECT_EQ(ellipsoidClamp(current_vec3, ref_vec), Vector2f(1.F, 1.F) * std::sqrt(2.F) / 2.F);
}

TEST(RectClamp, Vector2) {
  Vector2f current_vec1(2.F, 0.F);
  Vector2f current_vec2(0.F, 2.F);
  Vector2f ref_vec(1.F, 1.F);

  // Test rectClamp function
  EXPECT_EQ(rectClamp(current_vec1, ref_vec), Vector2f(1.F, 0.F));
  EXPECT_EQ(rectClamp(current_vec2, ref_vec), Vector2f(0.F, 1.F));
}

TEST(EllipsoidClamp, ZeroCase) {
  Vector3f current_vec(1.F, 1.F, 1.F);
  Vector3f ref_vec(0.F, 0.F, 0.F);

  // Test ellipsoidClamp function with failure case
  EXPECT_EQ(rectClamp(current_vec, ref_vec), Vector3f(0.F, 0.F, 0.F));
}
