/**
 * @file lola_frame.h
 *
 * This file is subject to the terms of the HTWK License.
 * A copy of this license is included in LICENSE.HTWK.txt.
 * (c) 2023 HTWK and NomadZ team
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include <msgpack.hpp>

#include "naobridge/nomadz.h"

struct LolaSensorFrame {
  float data[lbhNumOfSensorIds];
  float unused;
};

struct LolaActuatorFrame {
  float data[lbhNumOfActuatorIds];
  float unused;
  bool sonar[2];
};

class LolaFrameHandler {
public:
  LolaFrameHandler();
  const LolaSensorFrame& unpack(const char* const buffer, size_t size);
  std::pair<char*, size_t> pack();

  LolaActuatorFrame actuator_frame;
  LolaSensorFrame sensor_frame;

private:
  void initSensorFrame();
  void initActuatorFrame();

  std::map<std::string, std::vector<float*>> sensor_frame_positions;
  std::map<std::string, std::vector<float*>> actuator_frame_positions;
  msgpack::sbuffer buffer;
};
