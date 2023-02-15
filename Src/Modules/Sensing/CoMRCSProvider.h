/**
 * @file CoMRCSProvider.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/PatternGenRequest.h"

MODULE(CoMRCSProvider)
REQUIRES(RobotModel)
REQUIRES(PatternGenRequest)
PROVIDES_WITH_MODIFY(ActualCoMRCS)
LOADS_PARAMETER(bool, walkFixedCoM)
END_MODULE

class CoMRCSProvider : public CoMRCSProviderBase {
public:
  void update(ActualCoMRCS& actualCoMRCS);
};