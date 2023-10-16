/**
 * @file FsrDataProvider.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Sensing/FsrData.h"

#include "Core/RingBufferWithSum.h"

MODULE(FsrDataProvider)
REQUIRES(RobotModel)
REQUIRES(SensorData)
PROVIDES_WITH_MODIFY(FsrData)
DEFINES_PARAMETER(float, fsrWeightOffset, 2.0f)
END_MODULE

/**
 * @class ExpGroundContactDetector
 * A module for sensor data filtering.
 */
class FsrDataProvider : public FsrDataProviderBase {
public:
  /** Default constructor. */
  FsrDataProvider();

private:
  Vector2<> LFsrFL;
  Vector2<> LFsrFR;
  Vector2<> LFsrRL;
  Vector2<> LFsrRR;
  Vector2<> RFsrFL;
  Vector2<> RFsrFR;
  Vector2<> RFsrRL;
  Vector2<> RFsrRR;

  RingBufferWithSum<float, 10> fsrL;
  RingBufferWithSum<float, 10> fsrR;

  void update(FsrData& fsrData);
};
