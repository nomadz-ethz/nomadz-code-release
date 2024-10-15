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

#include "nao_lola_client/msgpack_parser.hpp"

#include <vector>
#include <map>
#include <string>
#include <memory>

#include <gtest/gtest.h>

#include "nao_lola_sensor_msgs/msg/joint_indexes.hpp"

const std::vector<int> STATUS = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3};
const std::vector<float> STIFFNESS = {0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8};
const std::vector<float> ACCELEROMETER = {-3.0656251907348633, -0.39278322458267212, -9.3214168548583984};
const std::vector<float> BATTERY = {0.9, 0.5, 0.0, 37.0};
const std::vector<float> CURRENT = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2};
const std::vector<float> TOUCH = {1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0};
const std::vector<float> FSR = {0.014380865730345249,
                                0.29265055060386658,
                                0.47892898321151733,
                                0.62120223045349121,
                                0.28502300381660461,
                                0.70163685083389282,
                                0.40598109364509583,
                                0.086648054420948029};
const std::vector<float> ANGLES = {0.037582572549581528, -0.35991066694259644};
const std::vector<float> POSITION = {
  0.59361600875854492,  0.49544000625610352,  1.2133520841598511,    0.33283615112304688,  0.76849198341369629,
  -0.16869807243347168, -0.39427995681762695, -0.50617814064025879,  0.297637939453125,    -0.34050607681274414,
  2.1506261825561523,   -1.1137261390686035,  -0.062852144241333008, -0.10733795166015625, -0.23014187812805176,
  2.1399722099304199,   -1.2056820392608643,  -0.10120201110839844,  1.0600361824035645,   0.2039799690246582,
  -0.46791195869445801, 0.066004037857055664, 0.47089600563049316,   0.010400056838989258, 0.011199951171875};
const std::vector<float> SONAR = {0.3, 1.3};
const std::vector<float> GYROSCOPE = {-0.00026631611399352551, -0.001065264455974102, 0.001065264455974102};
const std::vector<float> TEMPERATURE = {38.0, 38.0, 38.0, 38.0, 38.0, 38.0, 38.0, 33.0, 27.0, 27.0, 27.0, 27.0, 27.0,
                                        27.0, 27.0, 27.0, 27.0, 27.0, 38.0, 38.0, 38.0, 38.0, 38.0, 38.0, 39.0};
const std::vector<std::string> ROBOT_CONFIG = {"P0000073A07S94700012", "6.0.0", "P0000074A05S93M00061", "6.0.0"};

namespace nl_sensor_msgs = nao_lola_sensor_msgs::msg;
using nao_lola_client::MsgpackParser;

class TestMsgpackParser : public ::testing::Test {
public:
  // NOLINTBEGIN(misc-non-private-member-variables-in-classes)
  std::shared_ptr<MsgpackParser> parser;
  // NOLINTEND(misc-non-private-member-variables-in-classes)

protected:
  void SetUp() override {
    // A way of packing a hashmap containing floats and strings and other
    // data types are explained in this comment:
    // https://github.com/msgpack/msgpack-c/issues/651#issuecomment-365197261
    msgpack::zone z;

    std::map<std::string, msgpack::object> map{{"Status", msgpack::object(STATUS, z)},
                                               {"Stiffness", msgpack::object(STIFFNESS, z)},
                                               {"Accelerometer", msgpack::object(ACCELEROMETER, z)},
                                               {"Battery", msgpack::object(BATTERY, z)},
                                               {"Current", msgpack::object(CURRENT, z)},
                                               {"Touch", msgpack::object(TOUCH, z)},
                                               {"FSR", msgpack::object(FSR, z)},
                                               {"Angles", msgpack::object(ANGLES, z)},
                                               {"Position", msgpack::object(POSITION, z)},
                                               {"Sonar", msgpack::object(SONAR, z)},
                                               {"Gyroscope", msgpack::object(GYROSCOPE, z)},
                                               {"Temperature", msgpack::object(TEMPERATURE, z)},
                                               {"RobotConfig", msgpack::object(ROBOT_CONFIG, z)}};

    // serialize the buffer
    std::stringstream buffer;
    msgpack::pack(buffer, map);

    // deserialize the buffer
    // DO NOT use a std::string because serialized data may contain null characters
    char c;
    std::vector<char> packed;
    packed.reserve(1000);
    while (buffer.get(c)) {
      packed.push_back(c);
    }
    parser = std::make_shared<MsgpackParser>(packed.data(), packed.size());
  }
};

TEST_F(TestMsgpackParser, TestIMU) {
  nl_sensor_msgs::Imu imu = parser->getImu();
  EXPECT_NEAR(imu.accelerometer.x, -3.0656251907348633, 0.000001);
  EXPECT_NEAR(imu.accelerometer.y, -0.39278322458267212, 0.000001);
  EXPECT_NEAR(imu.accelerometer.z, -9.3214168548583984, 0.000001);

  EXPECT_NEAR(imu.gyroscope.x, -0.00026631611399352551, 0.000001);
  EXPECT_NEAR(imu.gyroscope.y, -0.001065264455974102, 0.000001);
  EXPECT_NEAR(imu.gyroscope.z, 0.001065264455974102, 0.000001);

  EXPECT_NEAR(imu.angle_roll, 0.037582572549581528, 0.000001);
  EXPECT_NEAR(imu.angle_pitch, -0.35991066694259644, 0.000001);
}

TEST_F(TestMsgpackParser, TestButtons) {
  nl_sensor_msgs::Buttons but = parser->getButtons();
  EXPECT_TRUE(but.chest);
  EXPECT_FALSE(but.head_front);
  EXPECT_TRUE(but.head_middle);
  EXPECT_FALSE(but.head_rear);
  EXPECT_TRUE(but.l_foot_bumper_left);
  EXPECT_FALSE(but.l_foot_bumper_right);
  EXPECT_FALSE(but.r_foot_bumper_left);
  EXPECT_TRUE(but.r_foot_bumper_right);
}

TEST_F(TestMsgpackParser, TestFSR) {
  nl_sensor_msgs::Fsr fsr = parser->getFsr();
  EXPECT_NEAR(fsr.l_foot_front_left, 0.014380865730345249, 0.000001);
  EXPECT_NEAR(fsr.l_foot_front_right, 0.29265055060386658, 0.000001);
  EXPECT_NEAR(fsr.l_foot_back_left, 0.47892898321151733, 0.000001);
  EXPECT_NEAR(fsr.l_foot_back_right, 0.62120223045349121, 0.000001);
  EXPECT_NEAR(fsr.r_foot_front_left, 0.28502300381660461, 0.000001);
  EXPECT_NEAR(fsr.r_foot_front_right, 0.70163685083389282, 0.000001);
  EXPECT_NEAR(fsr.r_foot_back_left, 0.40598109364509583, 0.000001);
  EXPECT_NEAR(fsr.r_foot_back_right, 0.086648054420948029, 0.000001);
}

TEST_F(TestMsgpackParser, TestJointData) {
  nl_sensor_msgs::JointData joint_data = parser->getJointData();
  EXPECT_NEAR(joint_data.positions.at(nl_sensor_msgs::JointIndexes::HEADYAW), 0.59361600875854492, 0.000001);
  EXPECT_NEAR(joint_data.positions.at(nl_sensor_msgs::JointIndexes::RHAND), 0.011199951171875, 0.00001);
  EXPECT_NEAR(joint_data.stiffnesses.at(nl_sensor_msgs::JointIndexes::HEADYAW), 0.3, 0.000001);
  EXPECT_NEAR(joint_data.stiffnesses.at(nl_sensor_msgs::JointIndexes::RHAND), 0.8, 0.00001);
  EXPECT_NEAR(joint_data.temperatures.at(nl_sensor_msgs::JointIndexes::HEADYAW), 38.0, 0.000001);
  EXPECT_NEAR(joint_data.temperatures.at(nl_sensor_msgs::JointIndexes::RHAND), 39.0, 0.00001);
  EXPECT_NEAR(joint_data.currents.at(nl_sensor_msgs::JointIndexes::HEADYAW), 0.1, 0.000001);
  EXPECT_NEAR(joint_data.currents.at(nl_sensor_msgs::JointIndexes::RHAND), 0.2, 0.00001);
  EXPECT_EQ(joint_data.statuses.at(nl_sensor_msgs::JointIndexes::HEADYAW), 1);
  EXPECT_EQ(joint_data.statuses.at(nl_sensor_msgs::JointIndexes::RHAND), 3);
}

TEST_F(TestMsgpackParser, TestSonar) {
  nl_sensor_msgs::Sonar snr = parser->getSonar();
  EXPECT_NEAR(snr.left, 0.3, 0.000001);
  EXPECT_NEAR(snr.right, 1.3, 0.000001);
}

TEST_F(TestMsgpackParser, TestBattery) {
  nl_sensor_msgs::Battery btr = parser->getBattery();
  // From Lola, charge has range 0.0 - 1.0, but in Battery msg, we have 0.0% - 100.0%.
  // So, we multiply by 100 below.
  EXPECT_NEAR(btr.charge, 0.9 * 100, 0.000001);
  EXPECT_NEAR(btr.current, 0.5, 0.000001);
  EXPECT_FALSE(btr.charging);
  EXPECT_NEAR(btr.temperature, 37.0, 0.000001);
}

TEST_F(TestMsgpackParser, TestRobotConfig) {
  nl_sensor_msgs::RobotConfig robot_config = parser->getRobotConfig();
  EXPECT_EQ(robot_config.body_id, "P0000073A07S94700012");
  EXPECT_EQ(robot_config.body_version, "6.0.0");
  EXPECT_EQ(robot_config.head_id, "P0000074A05S93M00061");
  EXPECT_EQ(robot_config.head_version, "6.0.0");
}
