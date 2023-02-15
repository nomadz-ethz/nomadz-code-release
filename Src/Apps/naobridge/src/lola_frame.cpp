/**
 * @file lola_frame.cpp
 *
 * This file is subject to the terms of the HTWK License.
 * A copy of this license is included in LICENSE.HTWK.txt.
 * (c) 2022 HTWK and NomadZ team
 */

#include "lola_frame.h"
#include "stl_ext.h"

using namespace std;

LolaFrameHandler::LolaFrameHandler() {
  initSensorFrame();
  initActuatorFrame();
}

void LolaFrameHandler::initSensorFrame() {
  auto& d = sensor_frame.data;
  auto& u = sensor_frame.unused;
  sensor_frame_positions = {
    {"Accelerometer", {&d[accXSensor], &d[accYSensor], &d[accZSensor]}},
    {"Gyroscope", {&d[gyroXSensor], &d[gyroYSensor], &d[gyroZSensor]}},
    // The order for battery values in Softbank's documentation is wrong.
    {"Battery", {&d[batteryChargeSensor], &u, &u, &u}},
    {"FSR",
     {&d[lFSRFrontLeftSensor],
      &d[lFSRFrontRightSensor],
      &d[lFSRRearLeftSensor],
      &d[lFSRRearRightSensor],
      &d[rFSRFrontLeftSensor],
      &d[rFSRFrontRightSensor],
      &d[rFSRRearLeftSensor],
      &d[rFSRRearRightSensor]}},
    {"Angles", {&d[angleXSensor], &d[angleYSensor]}},
    {"Sonar", {&d[lUsSensor], &d[rUsSensor]}},
    {"Touch",
     {&d[chestButtonSensor],
      &d[headFrontSensor],
      &d[headMiddleSensor],
      &d[headRearSensor],
      &d[lBumperLeftSensor],
      &d[lBumperRightSensor],
      &u,
      &u,
      &u,
      &d[rBumperLeftSensor],
      &d[rBumperRightSensor],
      &u,
      &u,
      &u}},
    {"Position", {&d[headYawPositionSensor],        &d[headPitchPositionSensor],     &d[lShoulderPitchPositionSensor],
                  &d[lShoulderRollPositionSensor],  &d[lElbowYawPositionSensor],     &d[lElbowRollPositionSensor],
                  &d[lWristYawPositionSensor],      &d[lHipYawPitchPositionSensor],  &d[lHipRollPositionSensor],
                  &d[lHipPitchPositionSensor],      &d[lKneePitchPositionSensor],    &d[lAnklePitchPositionSensor],
                  &d[lAnkleRollPositionSensor],     &d[rHipRollPositionSensor],      &d[rHipPitchPositionSensor],
                  &d[rKneePitchPositionSensor],     &d[rAnklePitchPositionSensor],   &d[rAnkleRollPositionSensor],
                  &d[rShoulderPitchPositionSensor], &d[rShoulderRollPositionSensor], &d[rElbowYawPositionSensor],
                  &d[rElbowRollPositionSensor],     &d[rWristYawPositionSensor],     &d[lHandPositionSensor],
                  &d[rHandPositionSensor]}},
    {"Current", {&d[headYawCurrentSensor],        &d[headPitchCurrentSensor],     &d[lShoulderPitchCurrentSensor],
                 &d[lShoulderRollCurrentSensor],  &d[lElbowYawCurrentSensor],     &d[lElbowRollCurrentSensor],
                 &d[lWristYawCurrentSensor],      &d[lHipYawPitchCurrentSensor],  &d[lHipRollCurrentSensor],
                 &d[lHipPitchCurrentSensor],      &d[lKneePitchCurrentSensor],    &d[lAnklePitchCurrentSensor],
                 &d[lAnkleRollCurrentSensor],     &d[rHipRollCurrentSensor],      &d[rHipPitchCurrentSensor],
                 &d[rKneePitchCurrentSensor],     &d[rAnklePitchCurrentSensor],   &d[rAnkleRollCurrentSensor],
                 &d[rShoulderPitchCurrentSensor], &d[rShoulderRollCurrentSensor], &d[rElbowYawCurrentSensor],
                 &d[rElbowRollCurrentSensor],     &d[rWristYawCurrentSensor],     &d[lHandCurrentSensor],
                 &d[rHandCurrentSensor]}},
    {"Temperature",
     {&d[headYawTemperatureSensor],        &d[headPitchTemperatureSensor],     &d[lShoulderPitchTemperatureSensor],
      &d[lShoulderRollTemperatureSensor],  &d[lElbowYawTemperatureSensor],     &d[lElbowRollTemperatureSensor],
      &d[lWristYawTemperatureSensor],      &d[lHipYawPitchTemperatureSensor],  &d[lHipRollTemperatureSensor],
      &d[lHipPitchTemperatureSensor],      &d[lKneePitchTemperatureSensor],    &d[lAnklePitchTemperatureSensor],
      &d[lAnkleRollTemperatureSensor],     &d[rHipRollTemperatureSensor],      &d[rHipPitchTemperatureSensor],
      &d[rKneePitchTemperatureSensor],     &d[rAnklePitchTemperatureSensor],   &d[rAnkleRollTemperatureSensor],
      &d[rShoulderPitchTemperatureSensor], &d[rShoulderRollTemperatureSensor], &d[rElbowYawTemperatureSensor],
      &d[rElbowRollTemperatureSensor],     &d[rWristYawTemperatureSensor],     &d[lHandTemperatureSensor],
      &d[rHandTemperatureSensor]}},
  };
}

void LolaFrameHandler::initActuatorFrame() {
  auto& d = actuator_frame.data;
  // auto& u = actuator_frame.unused;
  // auto& sonar = actuator_frame.sonar;
  actuator_frame_positions = {
    {"Chest", {&d[chestBoardLedRedActuator], &d[chestBoardLedGreenActuator], &d[chestBoardLedBlueActuator]}},
    {"LFoot", {&d[lFootLedRedActuator], &d[lFootLedGreenActuator], &d[lFootLedBlueActuator]}},
    {"RFoot", {&d[rFootLedRedActuator], &d[rFootLedGreenActuator], &d[rFootLedBlueActuator]}},
    {"Position", {&d[headYawPositionActuator],        &d[headPitchPositionActuator],     &d[lShoulderPitchPositionActuator],
                  &d[lShoulderRollPositionActuator],  &d[lElbowYawPositionActuator],     &d[lElbowRollPositionActuator],
                  &d[lWristYawPositionActuator],      &d[lHipYawPitchPositionActuator],  &d[lHipRollPositionActuator],
                  &d[lHipPitchPositionActuator],      &d[lKneePitchPositionActuator],    &d[lAnklePitchPositionActuator],
                  &d[lAnkleRollPositionActuator],     &d[rHipRollPositionActuator],      &d[rHipPitchPositionActuator],
                  &d[rKneePitchPositionActuator],     &d[rAnklePitchPositionActuator],   &d[rAnkleRollPositionActuator],
                  &d[rShoulderPitchPositionActuator], &d[rShoulderRollPositionActuator], &d[rElbowYawPositionActuator],
                  &d[rElbowRollPositionActuator],     &d[rWristYawPositionActuator],     &d[lHandPositionActuator],
                  &d[rHandPositionActuator]}},
    {"Stiffness", {&d[headYawHardnessActuator],        &d[headPitchHardnessActuator],     &d[lShoulderPitchHardnessActuator],
                   &d[lShoulderRollHardnessActuator],  &d[lElbowYawHardnessActuator],     &d[lElbowRollHardnessActuator],
                   &d[lWristYawHardnessActuator],      &d[lHipYawPitchHardnessActuator],  &d[lHipRollHardnessActuator],
                   &d[lHipPitchHardnessActuator],      &d[lKneePitchHardnessActuator],    &d[lAnklePitchHardnessActuator],
                   &d[lAnkleRollHardnessActuator],     &d[rHipRollHardnessActuator],      &d[rHipPitchHardnessActuator],
                   &d[rKneePitchHardnessActuator],     &d[rAnklePitchHardnessActuator],   &d[rAnkleRollHardnessActuator],
                   &d[rShoulderPitchHardnessActuator], &d[rShoulderRollHardnessActuator], &d[rElbowYawHardnessActuator],
                   &d[rElbowRollHardnessActuator],     &d[rWristYawHardnessActuator],     &d[lHandHardnessActuator],
                   &d[rHandHardnessActuator]}},
    {"LEar",
     {&d[earsLedLeft0DegActuator],
      &d[earsLedLeft36DegActuator],
      &d[earsLedLeft72DegActuator],
      &d[earsLedLeft108DegActuator],
      &d[earsLedLeft144DegActuator],
      &d[earsLedLeft180DegActuator],
      &d[earsLedLeft216DegActuator],
      &d[earsLedLeft252DegActuator],
      &d[earsLedLeft288DegActuator],
      &d[earsLedLeft324DegActuator]}},
    {"REar",
     {&d[earsLedRight324DegActuator],
      &d[earsLedRight288DegActuator],
      &d[earsLedRight252DegActuator],
      &d[earsLedRight216DegActuator],
      &d[earsLedRight180DegActuator],
      &d[earsLedRight144DegActuator],
      &d[earsLedRight108DegActuator],
      &d[earsLedRight72DegActuator],
      &d[earsLedRight36DegActuator],
      &d[earsLedRight0DegActuator]}},
    {"LEye", {&d[faceLedRedLeft45DegActuator],    &d[faceLedRedLeft0DegActuator],     &d[faceLedRedLeft315DegActuator],
              &d[faceLedRedLeft270DegActuator],   &d[faceLedRedLeft225DegActuator],   &d[faceLedRedLeft180DegActuator],
              &d[faceLedRedLeft135DegActuator],   &d[faceLedRedLeft90DegActuator],    &d[faceLedGreenLeft45DegActuator],
              &d[faceLedGreenLeft0DegActuator],   &d[faceLedGreenLeft315DegActuator], &d[faceLedGreenLeft270DegActuator],
              &d[faceLedGreenLeft225DegActuator], &d[faceLedGreenLeft180DegActuator], &d[faceLedGreenLeft135DegActuator],
              &d[faceLedGreenLeft90DegActuator],  &d[faceLedBlueLeft45DegActuator],   &d[faceLedBlueLeft0DegActuator],
              &d[faceLedBlueLeft315DegActuator],  &d[faceLedBlueLeft270DegActuator],  &d[faceLedBlueLeft225DegActuator],
              &d[faceLedBlueLeft180DegActuator],  &d[faceLedBlueLeft135DegActuator],  &d[faceLedBlueLeft90DegActuator]}},
    {"REye", {&d[faceLedRedRight0DegActuator],     &d[faceLedRedRight45DegActuator],    &d[faceLedRedRight90DegActuator],
              &d[faceLedRedRight135DegActuator],   &d[faceLedRedRight180DegActuator],   &d[faceLedRedRight225DegActuator],
              &d[faceLedRedRight270DegActuator],   &d[faceLedRedRight315DegActuator],   &d[faceLedGreenRight0DegActuator],
              &d[faceLedGreenRight45DegActuator],  &d[faceLedGreenRight90DegActuator],  &d[faceLedGreenRight135DegActuator],
              &d[faceLedGreenRight180DegActuator], &d[faceLedGreenRight225DegActuator], &d[faceLedGreenRight270DegActuator],
              &d[faceLedGreenRight315DegActuator], &d[faceLedBlueRight0DegActuator],    &d[faceLedBlueRight45DegActuator],
              &d[faceLedBlueRight90DegActuator],   &d[faceLedBlueRight135DegActuator],  &d[faceLedBlueRight180DegActuator],
              &d[faceLedBlueRight225DegActuator],  &d[faceLedBlueRight270DegActuator],  &d[faceLedBlueRight315DegActuator]}}
    // TODO (leds): FOOT & SKULL
    // TODO (integer): {"Sonar", {&sonar[0], &sonar[1]}},
  };
}

const LolaSensorFrame& LolaFrameHandler::unpack(const char* const buffer, size_t size) {
  msgpack::unpacker pac;
  pac.reserve_buffer(size);
  memcpy(pac.buffer(), buffer, size);
  pac.buffer_consumed(size);
  msgpack::object_handle oh;
  // No message
  if (!pac.next(oh))
    return sensor_frame;

  const auto& map = oh.get().via.map;
  auto* category = map.ptr;
  for (uint32_t i = 0; i < map.size; i++, category++) {
    if (auto ptr = find(sensor_frame_positions, category->key.as<string>())) {
      zip(*ptr, category->val.as<vector<float>>(), [](float* r, float f) {
        if (r != nullptr)
          *r = f;
      });
    }
  }

  // Replicate sonar values (FIXME: forward old value)
  for (int i = lUs1Sensor; i <= lUs9Sensor; ++i) {
    sensor_frame.data[i] = sensor_frame.data[lUsSensor];
  }
  for (int i = rUs1Sensor; i <= rUs9Sensor; ++i) {
    sensor_frame.data[i] = sensor_frame.data[rUsSensor];
  }

  // FIXME: These seem to be different between robots but consistent between reboots. Make sure you calibrate each robot
  // otherwise most filters (e.g. Madgwick) won't work correctly!
  // sensor_frame.imu.gyr.yaw *= 1.0764f;
  // sensor_frame.imu.gyr.pitch *= 1.f;
  // sensor_frame.imu.gyr.roll *= 0.965f;
  return sensor_frame;
}

pair<char*, size_t> LolaFrameHandler::pack() {
  buffer.clear();
  msgpack::packer<msgpack::sbuffer> pk(&buffer);
  pk.pack_map(actuator_frame_positions.size() +
              1); // the +1 is for the sonar which cannot be packed the same way as the actuator_frame_position
  for (const auto& kv : actuator_frame_positions) {
    pk.pack(kv.first);
    pk.pack_array(kv.second.size());
    for (float* val : kv.second) {
      pk.pack(*val);
    }
  }
  pk.pack("Sonar");
  pk.pack_array(2);
  pk.pack(actuator_frame.sonar[0]);
  pk.pack(actuator_frame.sonar[1]);
  return {buffer.data(), buffer.size()};
}
