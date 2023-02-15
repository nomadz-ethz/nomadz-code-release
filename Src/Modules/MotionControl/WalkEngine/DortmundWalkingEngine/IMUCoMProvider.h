/**
 * @file IMUCoMProvider.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
 */

#pragma once
#include <list>
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/DortmundWalkingEngine/StepData.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Core/Module/Module.h"

/**
 * @class IMUCoMProvider
 * Determines the actual position of the center of mass
 * using the orientation given by naoqi.
 */

MODULE(IMUCoMProvider)
REQUIRES(FootSteps)
REQUIRES(InertiaSensorData)
REQUIRES(WalkingEngineParams)
USES(WalkingInfo)
USES(TargetCoM)
PROVIDES_WITH_MODIFY(ActualCoM)
END_MODULE

class IMUCoMProvider : public IMUCoMProviderBase {
public:
  IMUCoMProvider(){};

  /** Destructor */
  ~IMUCoMProvider(void){};

  void update(ActualCoM& theActualCoM);

private:
  // typedef std::list<Footposition> FootList;

  /** List of target foot positions in world coordinate system filled with the
  foot positions found in theFootpositions. No more needed positions
  are deleted in every step cycle. */
  // FootList footPositions;
};
