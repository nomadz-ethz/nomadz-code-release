/**
 * @file InertiaSensorProvider.h
 *
 * Declares a module that provides InertiaSensorData from SensorData.
 * Added during the move to Dortmund Walking Engine to make our InertiaSensorData compatible.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Core/Module/Module.h"

MODULE(InertiaSensorProvider)
REQUIRES(SensorData)
PROVIDES_WITH_MODIFY(InertiaSensorData)
END_MODULE

/**
 * @class InertiaSensorProvider
 * A Dortmund compatibility module that copies the IMU stuff from SensorData into InertiaSensorData.
 */
class InertiaSensorProvider : public InertiaSensorProviderBase {
public:
  /**
   * Default constructor.
   */
  InertiaSensorProvider() {}

  void update(InertiaSensorData& inertiaSensorData);
};
