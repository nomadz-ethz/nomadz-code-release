// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nao_lola_client/msgpack_packer.hpp"

#include <string>
#include <vector>
#include <map>
#include <memory>

#include <gtest/gtest.h>
#include <msgpack.hpp>

#include "nao_lola_command_msgs/msg/joint_requests.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "nao_lola_client/lola_enums.hpp"

static std::vector<float> getFloatVector(std::string packed, std::string mapKey);
static std::vector<bool> getBoolVector(std::string packed, std::string mapKey);
static std::map<std::string, msgpack::object> unpack(std::string packed);

namespace LolaEnums = nao_lola_client::LolaEnums;
namespace cmd_msgs = nao_lola_command_msgs::msg;
using nao_lola_client::MsgpackPacker;

class TestMsgpackPacker : public ::testing::Test {
public:
  MsgpackPacker packer;
};

TEST_F(TestMsgpackPacker, TestJointData) {
  cmd_msgs::JointRequests command;
  command.indexes.push_back(cmd_msgs::JointIndexes::HEADYAW);
  command.positions.push_back(1.01);
  command.stiffnesses.push_back(0.7);
  command.indexes.push_back(cmd_msgs::JointIndexes::RHAND);
  command.positions.push_back(2.0);
  command.stiffnesses.push_back(0.3);

  packer.setJointRequests(command);
  std::string packed = packer.getPacked();

  std::vector<float> position(static_cast<int>(LolaEnums::Joint::NUM_JOINTS), 0);
  position.at(static_cast<int>(LolaEnums::Joint::HeadYaw)) = 1.01;
  position.at(static_cast<int>(LolaEnums::Joint::RHand)) = 2.0;
  EXPECT_EQ(getFloatVector(packed, "Position"), position);

  std::vector<float> stiffness(static_cast<int>(LolaEnums::Joint::NUM_JOINTS), 0);
  stiffness.at(static_cast<int>(LolaEnums::Joint::HeadYaw)) = 0.7;
  stiffness.at(static_cast<int>(LolaEnums::Joint::RHand)) = 0.3;
  EXPECT_EQ(getFloatVector(packed, "Stiffness"), stiffness);
}

TEST_F(TestMsgpackPacker, TestChestLed) {
  cmd_msgs::ChestLed chest_led;
  chest_led.color.r = 0.1;
  chest_led.color.g = 0.5;
  chest_led.color.b = 1.0;

  packer.setChestLed(chest_led);
  std::string packed = packer.getPacked();

  std::vector<float> expected{0.1, 0.5, 1.0};
  EXPECT_EQ(getFloatVector(packed, "Chest"), expected);
}

TEST_F(TestMsgpackPacker, TestLeftEarLeds) {
  cmd_msgs::EarLeds ear_leds;
  ear_leds.left_intensities[cmd_msgs::EarLeds::L0] = 0.1;
  ear_leds.left_intensities[cmd_msgs::EarLeds::L1] = 0.2;
  ear_leds.left_intensities[cmd_msgs::EarLeds::L2] = 0.3;
  ear_leds.left_intensities[cmd_msgs::EarLeds::L3] = 0.4;
  ear_leds.left_intensities[cmd_msgs::EarLeds::L4] = 0.5;
  ear_leds.left_intensities[cmd_msgs::EarLeds::L5] = 0.6;
  ear_leds.left_intensities[cmd_msgs::EarLeds::L6] = 0.7;
  ear_leds.left_intensities[cmd_msgs::EarLeds::L7] = 0.8;
  ear_leds.left_intensities[cmd_msgs::EarLeds::L8] = 0.9;
  ear_leds.left_intensities[cmd_msgs::EarLeds::L9] = 1.0;

  packer.setLeftEarLeds(ear_leds);
  std::string packed = packer.getPacked();

  std::vector<float> expected{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
  EXPECT_EQ(getFloatVector(packed, "LEar"), expected);
}

TEST_F(TestMsgpackPacker, TestRightEarLeds) {
  cmd_msgs::EarLeds ear_leds;
  ear_leds.right_intensities[cmd_msgs::EarLeds::R0] = 0.1;
  ear_leds.right_intensities[cmd_msgs::EarLeds::R1] = 0.2;
  ear_leds.right_intensities[cmd_msgs::EarLeds::R2] = 0.3;
  ear_leds.right_intensities[cmd_msgs::EarLeds::R3] = 0.4;
  ear_leds.right_intensities[cmd_msgs::EarLeds::R4] = 0.5;
  ear_leds.right_intensities[cmd_msgs::EarLeds::R5] = 0.6;
  ear_leds.right_intensities[cmd_msgs::EarLeds::R6] = 0.7;
  ear_leds.right_intensities[cmd_msgs::EarLeds::R7] = 0.8;
  ear_leds.right_intensities[cmd_msgs::EarLeds::R8] = 0.9;
  ear_leds.right_intensities[cmd_msgs::EarLeds::R9] = 1.0;

  packer.setRightEarLeds(ear_leds);
  std::string packed = packer.getPacked();

  std::vector<float> expected{1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1};
  EXPECT_EQ(getFloatVector(packed, "REar"), expected);
}

TEST_F(TestMsgpackPacker, TestLeftEyeLeds) {
  // Explanation of eye correspondence: http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#nao-v5-v4-and-v3-3
  cmd_msgs::EyeLeds eye_leds;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L0].r = 0.01;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L0].g = 0.02;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L0].b = 0.03;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L1].r = 0.04;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L1].g = 0.05;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L1].b = 0.06;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L2].r = 0.07;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L2].g = 0.08;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L2].b = 0.09;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L3].r = 0.10;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L3].g = 0.11;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L3].b = 0.12;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L4].r = 0.13;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L4].g = 0.14;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L4].b = 0.15;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L5].r = 0.16;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L5].g = 0.17;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L5].b = 0.18;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L6].r = 0.19;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L6].g = 0.20;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L6].b = 0.21;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L7].r = 0.22;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L7].g = 0.23;
  eye_leds.left_colors[cmd_msgs::EyeLeds::L7].b = 0.24;

  packer.setLeftEyeLeds(eye_leds);
  std::string packed = packer.getPacked();

  std::vector<float> expected{0.01, 0.04, 0.07, 0.10, 0.13, 0.16, 0.19, 0.22, 0.02, 0.05, 0.08, 0.11,
                              0.14, 0.17, 0.20, 0.23, 0.03, 0.06, 0.09, 0.12, 0.15, 0.18, 0.21, 0.24};
  EXPECT_EQ(getFloatVector(packed, "LEye"), expected);
}

TEST_F(TestMsgpackPacker, TestRightEyeLeds) {
  // Explanation of eye correspondence: http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#nao-v5-v4-and-v3-3
  cmd_msgs::EyeLeds eye_leds;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R0].r = 0.01;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R0].g = 0.02;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R0].b = 0.03;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R1].r = 0.04;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R1].g = 0.05;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R1].b = 0.06;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R2].r = 0.07;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R2].g = 0.08;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R2].b = 0.09;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R3].r = 0.10;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R3].g = 0.11;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R3].b = 0.12;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R4].r = 0.13;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R4].g = 0.14;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R4].b = 0.15;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R5].r = 0.16;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R5].g = 0.17;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R5].b = 0.18;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R6].r = 0.19;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R6].g = 0.20;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R6].b = 0.21;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R7].r = 0.22;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R7].g = 0.23;
  eye_leds.right_colors[cmd_msgs::EyeLeds::R7].b = 0.24;

  packer.setRightEyeLeds(eye_leds);
  std::string packed = packer.getPacked();

  std::vector<float> expected{0.22, 0.19, 0.16, 0.13, 0.10, 0.07, 0.04, 0.01, 0.23, 0.20, 0.17, 0.14,
                              0.11, 0.08, 0.05, 0.02, 0.24, 0.21, 0.18, 0.15, 0.12, 0.09, 0.06, 0.03};
  EXPECT_EQ(getFloatVector(packed, "REye"), expected);
}

TEST_F(TestMsgpackPacker, TestLeftFootLed) {
  cmd_msgs::FootLed foot_led;
  foot_led.left_color.r = 0.2;
  foot_led.left_color.g = 0.3;
  foot_led.left_color.b = 0.4;

  packer.setLeftFootLed(foot_led);
  std::string packed = packer.getPacked();

  std::vector<float> expected{0.2, 0.3, 0.4};
  EXPECT_EQ(getFloatVector(packed, "LFoot"), expected);
}

TEST_F(TestMsgpackPacker, TestRightFootLed) {
  cmd_msgs::FootLed foot_led;
  foot_led.right_color.r = 0.5;
  foot_led.right_color.g = 0.6;
  foot_led.right_color.b = 0.7;

  packer.setRightFootLed(foot_led);
  std::string packed = packer.getPacked();

  std::vector<float> expected{0.5, 0.6, 0.7};
  EXPECT_EQ(getFloatVector(packed, "RFoot"), expected);
}

TEST_F(TestMsgpackPacker, TestHeadLeds) {
  cmd_msgs::HeadLeds head_leds;
  head_leds.intensities[cmd_msgs::HeadLeds::B0] = 0.00;
  head_leds.intensities[cmd_msgs::HeadLeds::B1] = 0.01;
  head_leds.intensities[cmd_msgs::HeadLeds::B2] = 0.02;
  head_leds.intensities[cmd_msgs::HeadLeds::B3] = 0.03;
  head_leds.intensities[cmd_msgs::HeadLeds::B4] = 0.04;
  head_leds.intensities[cmd_msgs::HeadLeds::B5] = 0.05;
  head_leds.intensities[cmd_msgs::HeadLeds::B6] = 0.06;
  head_leds.intensities[cmd_msgs::HeadLeds::B7] = 0.07;
  head_leds.intensities[cmd_msgs::HeadLeds::B8] = 0.08;
  head_leds.intensities[cmd_msgs::HeadLeds::B9] = 0.09;
  head_leds.intensities[cmd_msgs::HeadLeds::B10] = 0.10;
  head_leds.intensities[cmd_msgs::HeadLeds::B11] = 0.11;

  packer.setHeadLeds(head_leds);
  std::string packed = packer.getPacked();

  std::vector<float> expected{0.11, 0.10, 0.09, 0.08, 0.07, 0.06, 0.05, 0.04, 0.03, 0.02, 0.01, 0.00};
  EXPECT_EQ(getFloatVector(packed, "Skull"), expected);
}

TEST_F(TestMsgpackPacker, TestSonarUsage) {
  cmd_msgs::SonarUsage sonar_usage;
  sonar_usage.left = true;
  sonar_usage.right = false;

  packer.setSonarUsage(sonar_usage);
  std::string packed = packer.getPacked();

  std::vector<bool> expected{true, false};
  EXPECT_EQ(getBoolVector(packed, "Sonar"), expected);
}

// Helper functions
static std::vector<float> getFloatVector(std::string packed, std::string mapKey) {
  std::map<std::string, msgpack::object> unpacked = unpack(packed);
  return unpacked.at(mapKey).as<std::vector<float>>();
}

static std::vector<bool> getBoolVector(std::string packed, std::string mapKey) {
  std::map<std::string, msgpack::object> unpacked = unpack(packed);
  return unpacked.at(mapKey).as<std::vector<bool>>();
}

static std::map<std::string, msgpack::object> unpack(std::string packed) {
  msgpack::object_handle oh = msgpack::unpack(packed.data(), packed.size());
  return oh.get().as<std::map<std::string, msgpack::object>>();
}
