/**
 * @file CoPProvider.h
 *
 * Declaration of class CoPProvider, calculates zmp_acc
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Infrastructure/SensorData.h"

MODULE(CoPProvider)
REQUIRES(SensorData)
REQUIRES(RobotModel)
USES(WalkingInfo)
PROVIDES_WITH_MODIFY(ZMPModel)
END_MODULE

class CoPProvider : public CoPProviderBase {
public:
private:
  void update(ZMPModel& zmpModel);
};
