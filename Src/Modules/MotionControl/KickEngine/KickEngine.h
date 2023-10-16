/**
 * @file KickEngine.h
 *
 * This file declares a module that creates the kicking motions.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once

#include "KickEngineData.h"
#include "KickEngineParameters.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Core/Module/Module.h"
#include "Core/Streams/InStreams.h"
#include "Representations/Infrastructure/KeyStates.h"
#include "Representations/Configuration/JointCalibration.h"

MODULE(KickEngine)
USES(JointRequest)
REQUIRES(DamageConfiguration)
REQUIRES(FrameInfo)
REQUIRES(InertiaSensorData)
REQUIRES(JointData)
REQUIRES(MassCalibration)
REQUIRES(MotionRequest)
REQUIRES(MotionSelection)
REQUIRES(KeyStates)
REQUIRES(RobotDimensions)
REQUIRES(RobotModel)
REQUIRES(BallModel)
REQUIRES(TorsoMatrix)
REQUIRES(JointCalibration)
PROVIDES_WITH_MODIFY_AND_OUTPUT(KickEngineOutput)
LOADS_PARAMETER(omniKickParameters, theOmniKickParameters)
END_MODULE

class KickEngine : public KickEngineBase {
private:
  KickEngineData data;
  bool compensate = false;
  bool compensated = false;
  int boostState = 0;
  unsigned timeSinceLastPhase = 0;
  KickRequest lastValidKickRequest;

  std::vector<KickEngineParameters> params;

public:
  KickEngine();

  void update(KickEngineOutput& kickEngineOutput) override;
};
