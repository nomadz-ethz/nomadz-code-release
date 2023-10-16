/**
 * @file InertiaSensorProvider.cpp
 *
 * Defines a module that provides joint angles as if all joint calibration signs were +1.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include "InertiaSensorProvider.h"

void InertiaSensorProvider::update(InertiaSensorData& inertiaSensorData) {
  inertiaSensorData.gyro.x = theSensorData.data[SensorData::gyroX];
  inertiaSensorData.gyro.y = theSensorData.data[SensorData::gyroY];
  inertiaSensorData.gyro.z = theSensorData.data[SensorData::gyroZ]; // Aldebarans z-gyron is negated for some reason...

  inertiaSensorData.acc.x = theSensorData.data[SensorData::accX];
  if (inertiaSensorData.acc.x != InertiaSensorData::off) {
    inertiaSensorData.acc.x *= -1;
  }
  inertiaSensorData.acc.y = theSensorData.data[SensorData::accY];
  if (inertiaSensorData.acc.y != InertiaSensorData::off) {
    inertiaSensorData.acc.y *= -1;
  }
  inertiaSensorData.acc.z = theSensorData.data[SensorData::accZ];
  if (inertiaSensorData.acc.z != InertiaSensorData::off) {
    inertiaSensorData.acc.z *= -1;
  }

  inertiaSensorData.angle.x = theSensorData.data[SensorData::angleX];
  inertiaSensorData.angle.y = theSensorData.data[SensorData::angleY];

  inertiaSensorData.calibrated = true;
}

MAKE_MODULE(InertiaSensorProvider, Motion Infrastructure)
