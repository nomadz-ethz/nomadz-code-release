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

#include <map>
#include <string>
#include <vector>

#include "nao_lola_client/lola_enums.hpp"
#include "nao_lola_client/sensor_index_conversion.hpp"

namespace nao_lola_client {

  MsgpackParser::MsgpackParser(char data[], int size, rclcpp::Time time_stamp) : time_stamp_(time_stamp) {
    oh_ = msgpack::unpack(data, size);

    unpacked_ = oh_.get().as<std::map<std::string, msgpack::object>>();
  }

  std::vector<float> MsgpackParser::getSensorData(const std::string& type) const {
    return unpacked_.at(type).as<std::vector<float>>();
  }

  nao_lola_sensor_msgs::msg::Imu MsgpackParser::getImu() const {
    nao_lola_sensor_msgs::msg::Imu imu;
    imu.header.stamp = time_stamp_;
    std::vector<float> acc = unpacked_.at("Accelerometer").as<std::vector<float>>();
    imu.accelerometer.x = acc.at(static_cast<int>(LolaEnums::Accelerometer::X));
    imu.accelerometer.y = acc.at(static_cast<int>(LolaEnums::Accelerometer::Y));
    imu.accelerometer.z = acc.at(static_cast<int>(LolaEnums::Accelerometer::Z));

    std::vector<float> gyro = unpacked_.at("Gyroscope").as<std::vector<float>>();
    imu.gyroscope.x = gyro.at(static_cast<int>(LolaEnums::Gyroscope::X));
    imu.gyroscope.y = gyro.at(static_cast<int>(LolaEnums::Gyroscope::Y));
    imu.gyroscope.z = gyro.at(static_cast<int>(LolaEnums::Gyroscope::Z));

    std::vector<float> angle = unpacked_.at("Angles").as<std::vector<float>>();
    imu.angle_roll = angle.at(static_cast<int>(LolaEnums::Angles::X));
    imu.angle_pitch = angle.at(static_cast<int>(LolaEnums::Angles::Y));
    return imu;
  }

  nao_lola_sensor_msgs::msg::Buttons MsgpackParser::getButtons() const {
    nao_lola_sensor_msgs::msg::Buttons but;
    but.header.stamp = time_stamp_;
    std::vector<float> vec = unpacked_.at("Touch").as<std::vector<float>>();
    but.chest = vec.at(static_cast<int>(LolaEnums::Touch::ChestBoard_Button)) != 0.F;
    but.head_front = vec.at(static_cast<int>(LolaEnums::Touch::Head_Touch_Front)) != 0.F;
    but.head_middle = vec.at(static_cast<int>(LolaEnums::Touch::Head_Touch_Middle)) != 0.F;
    but.head_rear = vec.at(static_cast<int>(LolaEnums::Touch::Head_Touch_Rear)) != 0.F;
    but.l_foot_bumper_left = vec.at(static_cast<int>(LolaEnums::Touch::LFoot_Bumper_Left)) != 0.F;
    but.l_foot_bumper_right = vec.at(static_cast<int>(LolaEnums::Touch::LFoot_Bumper_Right)) != 0.F;
    but.r_foot_bumper_left = vec.at(static_cast<int>(LolaEnums::Touch::RFoot_Bumper_Left)) != 0.F;
    but.r_foot_bumper_right = vec.at(static_cast<int>(LolaEnums::Touch::RFoot_Bumper_Right)) != 0.F;
    return but;
  }

  nao_lola_sensor_msgs::msg::Fsr MsgpackParser::getFsr() const {
    nao_lola_sensor_msgs::msg::Fsr fsr;
    fsr.header.stamp = time_stamp_;
    std::vector<float> vec = unpacked_.at("FSR").as<std::vector<float>>();
    fsr.l_foot_front_left = vec.at(static_cast<int>(LolaEnums::FSR::LFoot_FrontLeft));
    fsr.l_foot_front_right = vec.at(static_cast<int>(LolaEnums::FSR::LFoot_FrontRight));
    fsr.l_foot_back_left = vec.at(static_cast<int>(LolaEnums::FSR::LFoot_RearLeft));
    fsr.l_foot_back_right = vec.at(static_cast<int>(LolaEnums::FSR::LFoot_RearRight));
    fsr.r_foot_front_left = vec.at(static_cast<int>(LolaEnums::FSR::RFoot_FrontLeft));
    fsr.r_foot_front_right = vec.at(static_cast<int>(LolaEnums::FSR::RFoot_FrontRight));
    fsr.r_foot_back_left = vec.at(static_cast<int>(LolaEnums::FSR::RFoot_RearLeft));
    fsr.r_foot_back_right = vec.at(static_cast<int>(LolaEnums::FSR::RFoot_RearRight));
    return fsr;
  }

  nao_lola_sensor_msgs::msg::JointData MsgpackParser::getJointData() const {
    nao_lola_sensor_msgs::msg::JointData joint_data;
    joint_data.header.stamp = time_stamp_;
    std::vector<float> positions = unpacked_.at("Position").as<std::vector<float>>();
    std::vector<float> stiffnesses = unpacked_.at("Stiffness").as<std::vector<float>>();
    std::vector<float> temperatures = unpacked_.at("Temperature").as<std::vector<float>>();
    std::vector<float> currents = unpacked_.at("Current").as<std::vector<float>>();
    std::vector<float> statuses = unpacked_.at("Status").as<std::vector<float>>();
    for (int i = 0; i < static_cast<int>(LolaEnums::Joint::NUM_JOINTS); ++i) {
      const int msg_index = IndexConversion::joint_lola_to_msg.at(static_cast<LolaEnums::Joint>(i));
      joint_data.positions.at(msg_index) = positions.at(i);
      joint_data.stiffnesses.at(msg_index) = stiffnesses.at(i);
      joint_data.temperatures.at(msg_index) = temperatures.at(i);
      joint_data.currents.at(msg_index) = currents.at(i);
      joint_data.statuses.at(msg_index) = static_cast<int32_t>(statuses.at(i));
    }
    return joint_data;
  }

  nao_lola_sensor_msgs::msg::Sonar MsgpackParser::getSonar() const {
    nao_lola_sensor_msgs::msg::Sonar snr;
    snr.header.stamp = time_stamp_;
    std::vector<float> vec = unpacked_.at("Sonar").as<std::vector<float>>();
    snr.left = vec.at(static_cast<int>(LolaEnums::Sonar::Left));
    snr.right = vec.at(static_cast<int>(LolaEnums::Sonar::Right));
    return snr;
  }

  nao_lola_sensor_msgs::msg::Battery MsgpackParser::getBattery() const {
    nao_lola_sensor_msgs::msg::Battery btr;
    btr.header.stamp = time_stamp_;
    std::vector<float> vec = unpacked_.at("Battery").as<std::vector<float>>();
    // Convert charge to [0% - 100%]
    btr.charge = vec.at(static_cast<int>(LolaEnums::Battery::Charge)) * 100.F;
    btr.current = vec.at(static_cast<int>(LolaEnums::Battery::Current));
    btr.temperature = vec.at(static_cast<int>(LolaEnums::Battery::Temperature));

    // Check whether robot is charging, with BHuman's equation used as reference:
    // https://github.com/bhuman/BHumanCodeRelease/tree/coderelease2019/Src/Modules/Infrastructure/NaoProvider/NaoProvider.cpp#L320
    const float status = vec.at(static_cast<int>(LolaEnums::Battery::Status));
    btr.charging = ((static_cast<int16_t>(status) & 0x80) != 0);

    return btr;
  }

  nao_lola_sensor_msgs::msg::RobotConfig MsgpackParser::getRobotConfig() const {
    nao_lola_sensor_msgs::msg::RobotConfig cfg;
    std::vector<std::string> vec = unpacked_.at("RobotConfig").as<std::vector<std::string>>();
    cfg.body_id = vec.at(static_cast<int>(LolaEnums::RobotConfig::Body_BodyId));
    cfg.body_version = vec.at(static_cast<int>(LolaEnums::RobotConfig::Body_Version));
    cfg.head_id = vec.at(static_cast<int>(LolaEnums::RobotConfig::Head_FullHeadId));
    cfg.head_version = vec.at(static_cast<int>(LolaEnums::RobotConfig::Head_Version));
    return cfg;
  }
} // namespace nao_lola_client
