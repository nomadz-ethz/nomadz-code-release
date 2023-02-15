/**
 * @file AutoCalibrator.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/KinematicOutput.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Infrastructure/JointData.h"
#include <algorithm>

MODULE(AutoCalibrator)

REQUIRES(FrameInfo)
USES(WalkingInfo)
USES(SpeedInfo)
REQUIRES(WalkingEngineParams)
REQUIRES(RobotDimensions)
REQUIRES(MassCalibration)
REQUIRES(RobotModel)
REQUIRES(ZMPModel)
REQUIRES(InertiaSensorData)
USES(JointRequest)
USES(KinematicOutput)
PROVIDES_WITH_MODIFY(WalkCalibration)
LOADS_PARAMETER(float, posErrAlpha)
LOADS_PARAMETER(float, rotErrAlpha)
LOADS_PARAMETER(float, maxPosError)
LOADS_PARAMETER(float, maxOriError)
END_MODULE

class AutoCalibrator : public AutoCalibratorBase {
public:
  AutoCalibrator(void);
  ~AutoCalibrator(void);

private:
  void update(WalkCalibration& walkCalibration);
  void init(WalkCalibration& walkCalibration);
  void saveCalibration(WalkCalibration& walkCalibration);
  void loadCalibration(WalkCalibration& walkCalibration);
  Vector3<> filteredPosErr, filteredOriErr, lastPosCorrection;
  RotationMatrix lastOriCorrection;

  Vector3<> desiredPositionOffset, desiredRotationOffset;
  unsigned timeCalibrationStart;
  bool calibrate;
  bool calibrationLoaded;
  bool calibrationOK;
  unsigned timeStampCalibrationOK;
};
