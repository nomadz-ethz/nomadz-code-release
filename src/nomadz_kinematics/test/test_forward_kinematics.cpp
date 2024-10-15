#include <gtest/gtest.h>

#include "nomadz_kinematics/forward_kinematics.hpp"

using nomadz_kinematics::torsoFromFinger;
using nomadz_kinematics::torsoFromFoot;
using nomadz_kinematics::torsoFromHead;

TEST(ForwardKinematics, testForwardKinematicsLeg) {
  Eigen::Affine3f left_foot_pose = torsoFromFoot(true, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F);
  Eigen::Affine3f right_foot_pose = torsoFromFoot(false, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F);
  EXPECT_FLOAT_EQ(left_foot_pose.translation().x(), 0.F);
  EXPECT_FLOAT_EQ(left_foot_pose.translation().y(), 0.05F);
  EXPECT_FLOAT_EQ(left_foot_pose.translation().z(), -0.2879F);
  EXPECT_FLOAT_EQ(right_foot_pose.translation().x(), 0.F);
  EXPECT_FLOAT_EQ(right_foot_pose.translation().y(), -0.05F);
  EXPECT_FLOAT_EQ(left_foot_pose.translation().z(), -0.2879F);
}

TEST(ForwardKinematics, testForwardKinematicsArm) {
  Eigen::Affine3f left_hand_pose = torsoFromFinger(true, 0.F, 0.F, 0.F, 0.F, 0.F);
  Eigen::Affine3f right_hand_pose = torsoFromFinger(false, 0.F, 0.F, 0.F, 0.F, 0.F);
  EXPECT_FLOAT_EQ(left_hand_pose.translation().x(), 0.2187F);
  EXPECT_FLOAT_EQ(left_hand_pose.translation().y(), 0.113F);
  EXPECT_FLOAT_EQ(left_hand_pose.translation().z(), 0.08769F);
  EXPECT_FLOAT_EQ(right_hand_pose.translation().x(), 0.2187F);
  EXPECT_FLOAT_EQ(right_hand_pose.translation().y(), -0.113F);
  EXPECT_FLOAT_EQ(right_hand_pose.translation().z(), 0.08769F);
}

TEST(ForwardKinematics, testForwardKinematicsHead) {
  Eigen::Affine3f head_pose = torsoFromHead(0.F, 0.F);
  EXPECT_FLOAT_EQ(head_pose.translation().x(), 0.F);
  EXPECT_FLOAT_EQ(head_pose.translation().y(), 0.F);
  EXPECT_FLOAT_EQ(head_pose.translation().z(), 0.1265F);
}
