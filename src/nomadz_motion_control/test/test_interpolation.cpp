#include <gtest/gtest.h>

#include "nomadz_motion_control/interpolation.hpp"
#include "nomadz_motion_control/joint_requests.hpp"

using nomadz_motion_control::JointRequestInterpolator;
using nomadz_motion_control::JointRequests;

TEST(testJointRequestInterpolator, NoInterpolation) {
  JointRequestInterpolator interpolator;

  JointRequests joint_request_1;
  JointRequests joint_request_2;

  for (size_t i = 0; i < joint_request_1.positions.values.size(); ++i) {
    joint_request_1.positions.values[i] = static_cast<float>(i);
    joint_request_1.stiffnesses.values[i] = static_cast<float>(i);
    joint_request_1.joint_ignore[i] = (i % 2) == 0;
    joint_request_2.positions.values[i] = static_cast<float>(i);
    joint_request_2.stiffnesses.values[i] = static_cast<float>(i);
    joint_request_2.joint_ignore[i] = (i % 2) == 0;
  }

  interpolator.process(joint_request_1, false);

  for (size_t i = 0; i < joint_request_1.positions.values.size(); ++i) {
    EXPECT_EQ(joint_request_1.positions.values[i], joint_request_2.positions.values[i]);
    EXPECT_EQ(joint_request_1.stiffnesses.values[i], joint_request_2.stiffnesses.values[i]);
  }

  for (size_t i = 0; i < joint_request_1.positions.values.size(); ++i) {
    joint_request_1.positions.values[i] = 2 * static_cast<float>(i);
    joint_request_1.stiffnesses.values[i] = 2 * static_cast<float>(i);
    joint_request_1.joint_ignore[i] = (i % 2) == 0;
    joint_request_2.positions.values[i] = 2 * static_cast<float>(i);
    joint_request_2.stiffnesses.values[i] = 2 * static_cast<float>(i);
    joint_request_2.joint_ignore[i] = (i % 2) == 0;
  }

  interpolator.process(joint_request_1, false);

  for (size_t i = 0; i < joint_request_1.positions.values.size(); ++i) {
    EXPECT_EQ(joint_request_1.positions.values[i], joint_request_2.positions.values[i]);
    EXPECT_EQ(joint_request_1.stiffnesses.values[i], joint_request_2.stiffnesses.values[i]);
    EXPECT_EQ(joint_request_1.joint_ignore[i], joint_request_2.joint_ignore[i]);
  }
}

TEST(testJointRequestInterpolator, InterpolationToValue) {
  JointRequestInterpolator interpolator;

  JointRequests initial;
  JointRequests target;
  JointRequests tmp;

  for (size_t i = 0; i < initial.positions.values.size(); ++i) {
    initial.positions.values[i] = static_cast<float>(i);
    initial.stiffnesses.values[i] = static_cast<float>(i);
    initial.joint_ignore[i] = (i % 2) == 0;
    target.positions.values[i] = 2 * static_cast<float>(i) + 1;
    target.stiffnesses.values[i] = 2 * static_cast<float>(i) + 1;
    target.joint_ignore[i] = (i % 2) == 1;
  }

  interpolator.process(initial, false);

  for (unsigned int i = 0; i < 50; ++i) {
    tmp = target;
    interpolator.process(tmp, i == 0);

    if (i < 49) {
      for (size_t j = 0; j < tmp.positions.values.size(); ++j) {
        EXPECT_EQ(tmp.joint_ignore[j], true);
      }
    }
  }

  for (size_t i = 0; i < tmp.positions.values.size(); ++i) {
    EXPECT_NEAR(tmp.positions.values[i], target.positions.values[i], 1e-6);
    EXPECT_NEAR(tmp.stiffnesses.values[i], target.stiffnesses.values[i], 1e-6);
    EXPECT_EQ(tmp.joint_ignore[i], target.joint_ignore[i]);
  }

  tmp = target;
  interpolator.process(tmp, false);
  for (size_t i = 0; i < tmp.positions.values.size(); ++i) {
    EXPECT_NEAR(tmp.positions.values[i], target.positions.values[i], 1e-6);
    EXPECT_NEAR(tmp.stiffnesses.values[i], target.stiffnesses.values[i], 1e-6);
    EXPECT_EQ(tmp.joint_ignore[i], target.joint_ignore[i]);
  }

  tmp = initial;
  interpolator.process(tmp, false);
  for (size_t i = 0; i < tmp.positions.values.size(); ++i) {
    EXPECT_NEAR(tmp.positions.values[i], initial.positions.values[i], 1e-6);
    EXPECT_NEAR(tmp.stiffnesses.values[i], initial.stiffnesses.values[i], 1e-6);
    EXPECT_EQ(tmp.joint_ignore[i], initial.joint_ignore[i]);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
