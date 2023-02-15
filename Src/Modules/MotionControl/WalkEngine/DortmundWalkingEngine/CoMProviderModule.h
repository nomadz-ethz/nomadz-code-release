/**
 * @file CoMProviderModule.h
 *
 * Module wrapper for the CoMProvider
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Core/Module/Module.h"
#include "CoMProvider.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/ActualCoM.h"

MODULE(CoMProviderModule)
// REQUIRES(JointAngles)
// REQUIRES(FootSteps)
// REQUIRES(RobotModel)
REQUIRES(ActualCoMRCS)
USES(WalkingInfo)
// USES(JointRequest),
// REQUIRES(WalkingEngineParams),
PROVIDES_WITH_MODIFY(ActualCoM)
PROVIDES_WITH_MODIFY(ActualCoMFLIPM)
END_MODULE

class CoMProviderModule : public CoMProviderModuleBase {
public:
  CoMProviderModule();

  CoMProvider controller;

  void update(ActualCoM& theActualCoM);
  void update(ActualCoMFLIPM& theActualCoMFLIPM);
};
