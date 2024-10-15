#include <random>
#include <gtest/gtest.h>

#include "nomadz_kinematics/forward_kinematics.hpp"
#include "nomadz_kinematics/inverse_kinematics.hpp"

constexpr float EPSILON = 2e-3;
using nomadz_kinematics::inverseTorsoFromFoot;
using nomadz_kinematics::inverseTorsoFromHead;
using nomadz_kinematics::torsoFromFinger;
using nomadz_kinematics::torsoFromFoot;
using nomadz_kinematics::torsoFromHead;

TEST(InverseKinematics, testInverseKinematicsLegDefault) {
  Eigen::Affine3f left_foot_pose = torsoFromFoot(true, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F);
  Eigen::Affine3f right_foot_pose = torsoFromFoot(false, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F);
  float hip_yaw_pitch = 0.F;
  float hip_roll = 0.F;
  float hip_pitch = 0.F;
  float knee_pitch = 0.F;
  float ankle_pitch = 0.F;
  float ankle_roll = 0.F;
  inverseTorsoFromFoot(true, left_foot_pose, hip_yaw_pitch, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll);
  EXPECT_FLOAT_EQ(hip_yaw_pitch, 0.F);
  EXPECT_FLOAT_EQ(hip_roll, 0.F);
  EXPECT_FLOAT_EQ(hip_pitch, 0.F);
  EXPECT_FLOAT_EQ(knee_pitch, 0.F);
  EXPECT_FLOAT_EQ(ankle_pitch, 0.F);
  EXPECT_FLOAT_EQ(ankle_roll, 0.F);
  inverseTorsoFromFoot(false, right_foot_pose, hip_yaw_pitch, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll);
  EXPECT_FLOAT_EQ(hip_yaw_pitch, 0.F);
  EXPECT_FLOAT_EQ(hip_roll, 0.F);
  EXPECT_FLOAT_EQ(hip_pitch, 0.F);
  EXPECT_FLOAT_EQ(knee_pitch, 0.F);
  EXPECT_FLOAT_EQ(ankle_pitch, 0.F);
  EXPECT_FLOAT_EQ(ankle_roll, 0.F);
}

TEST(InverseKinematics, testInverseKinematicsLegSpecialCase) {
  std::mt19937 gen(0);
  std::uniform_real_distribution<> dis(-1.0, 1.0);
  for (int i = 0; i < 10; i++) {
    std::array<float, 6> random_values;
    for (unsigned int i = 0; i < 6; ++i) {
      random_values[i] = static_cast<float>(dis(gen));
    }
    random_values[3] = std::abs(random_values[3]);
    random_values[4] = -(random_values[2] + random_values[3]);
    Eigen::Affine3f left_foot_pose = torsoFromFoot(
      true, random_values[0], random_values[1], random_values[2], random_values[3], random_values[4], random_values[5]);
    Eigen::Affine3f right_foot_pose = torsoFromFoot(
      false, random_values[0], random_values[1], random_values[2], random_values[3], random_values[4], random_values[5]);
    float hip_yaw_pitch_result = 0.F;
    float hip_roll_result = 0.F;
    float hip_pitch_result = 0.F;
    float knee_pitch_result = 0.F;
    float ankle_pitch_result = 0.F;
    float ankle_roll_result = 0.F;

    inverseTorsoFromFoot(true,
                         left_foot_pose,
                         hip_yaw_pitch_result,
                         hip_roll_result,
                         hip_pitch_result,
                         knee_pitch_result,
                         ankle_pitch_result,
                         ankle_roll_result);
    ASSERT_NEAR(hip_yaw_pitch_result, random_values[0], EPSILON);
    ASSERT_NEAR(hip_roll_result, random_values[1], EPSILON);
    ASSERT_NEAR(hip_pitch_result, random_values[2], EPSILON);
    ASSERT_NEAR(knee_pitch_result, random_values[3], EPSILON);
    ASSERT_NEAR(ankle_pitch_result, random_values[4], EPSILON);
    ASSERT_NEAR(ankle_roll_result, random_values[5], EPSILON);
    inverseTorsoFromFoot(false,
                         right_foot_pose,
                         hip_yaw_pitch_result,
                         hip_roll_result,
                         hip_pitch_result,
                         knee_pitch_result,
                         ankle_pitch_result,
                         ankle_roll_result);
    ASSERT_NEAR(hip_yaw_pitch_result, random_values[0], EPSILON);
    ASSERT_NEAR(hip_roll_result, random_values[1], EPSILON);
    ASSERT_NEAR(hip_pitch_result, random_values[2], EPSILON);
    ASSERT_NEAR(knee_pitch_result, random_values[3], EPSILON);
    ASSERT_NEAR(ankle_pitch_result, random_values[4], EPSILON);
    ASSERT_NEAR(ankle_roll_result, random_values[5], EPSILON);
  }
}

TEST(InverseKinematics, testInverseKinematicsLeg) {
  std::mt19937 gen(0);
  std::uniform_real_distribution<> dis(-1.0, 1.0);
  for (int i = 0; i < 1000; i++) {
    std::array<float, 6> random_values;
    for (unsigned int i = 0; i < 6; ++i) {
      random_values[i] = static_cast<float>(dis(gen));
    }
    random_values[3] = std::abs(random_values[3]);
    Eigen::Affine3f left_foot_pose = torsoFromFoot(
      true, random_values[0], random_values[1], random_values[2], random_values[3], random_values[4], random_values[5]);
    Eigen::Affine3f right_foot_pose = torsoFromFoot(
      false, random_values[0], random_values[1], random_values[2], random_values[3], random_values[4], random_values[5]);
    float hip_yaw_pitch_result = 0.F;
    float hip_roll_result = 0.F;
    float hip_pitch_result = 0.F;
    float knee_pitch_result = 0.F;
    float ankle_pitch_result = 0.F;
    float ankle_roll_result = 0.F;

    inverseTorsoFromFoot(true,
                         left_foot_pose,
                         hip_yaw_pitch_result,
                         hip_roll_result,
                         hip_pitch_result,
                         knee_pitch_result,
                         ankle_pitch_result,
                         ankle_roll_result);
    ASSERT_NEAR(hip_yaw_pitch_result, random_values[0], EPSILON);
    ASSERT_NEAR(hip_roll_result, random_values[1], EPSILON);
    ASSERT_NEAR(hip_pitch_result, random_values[2], EPSILON);
    ASSERT_NEAR(knee_pitch_result, random_values[3], EPSILON);
    ASSERT_NEAR(ankle_pitch_result, random_values[4], EPSILON);
    ASSERT_NEAR(ankle_roll_result, random_values[5], EPSILON);
    inverseTorsoFromFoot(false,
                         right_foot_pose,
                         hip_yaw_pitch_result,
                         hip_roll_result,
                         hip_pitch_result,
                         knee_pitch_result,
                         ankle_pitch_result,
                         ankle_roll_result);
    ASSERT_NEAR(hip_yaw_pitch_result, random_values[0], EPSILON);
    ASSERT_NEAR(hip_roll_result, random_values[1], EPSILON);
    ASSERT_NEAR(hip_pitch_result, random_values[2], EPSILON);
    ASSERT_NEAR(knee_pitch_result, random_values[3], EPSILON);
    ASSERT_NEAR(ankle_pitch_result, random_values[4], EPSILON);
    ASSERT_NEAR(ankle_roll_result, random_values[5], EPSILON);
  }
}

TEST(InverseKinematics, testInverseKinematicsHead) {
  std::mt19937 gen(0);
  std::uniform_real_distribution<> dis(-1.0, 1.0);
  for (int i = 0; i < 50; i++) {
    std::array<float, 2> random_values;
    for (unsigned int i = 0; i < 2; ++i) {
      random_values[i] = static_cast<float>(dis(gen));
    }
    Eigen::Affine3f head_pose = torsoFromHead(random_values[0], random_values[1]);
    float head_yaw_result = 0.F;
    float head_pitch_result = 0.F;
    inverseTorsoFromHead(head_pose, head_yaw_result, head_pitch_result);
    ASSERT_NEAR(head_yaw_result, random_values[0], EPSILON);
    ASSERT_NEAR(head_pitch_result, random_values[1], EPSILON);
  }
}
