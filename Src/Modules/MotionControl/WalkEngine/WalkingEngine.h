/**
 * @file WalkingEngine.cpp
 *
 * This file implements a module that is a wrapper for the UNSW walk generator.
 *
 * This file is subject to the terms of the BHuman 2017 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Thomas Röfer
 */

#pragma once

#include "Core/Math/Rotation.h"

#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkGeneratorData.h"
#include "Representations/MotionControl/PlannedSteps.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
//#include "Representations/MotionControl/WalkKickGenerator.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/OrientationData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Core/Module/Module.h"
#ifdef __INTELLISENSE__
#define INTELLISENSE_PREFIX WalkingEngine::
#endif

MODULE(WalkingEngine)
REQUIRES(FallDownState)
REQUIRES(FrameInfo)
REQUIRES(GroundContactState)
REQUIRES(InertiaSensorData)
REQUIRES(MotionRequest)
REQUIRES(MotionSelection)
USES(OdometryData)
REQUIRES(RobotModel)
REQUIRES(WalkGeneratorData)
REQUIRES(PlannedSteps)
REQUIRES(OrientationData)
PROVIDES_WITH_MODIFY(WalkingEngineOutput)
DEFINES_PARAMETER(int, minTimeInStandBeforeLeaving, (750)) /**< The minimum time in stand before leaving is possible. */
END_MODULE

/**
 * The wrapper for the UNSW walk generator. It is mainly implemented as a state machine,
 * because the original dribble engine is implemented here as well.
 */
class WalkingEngine : public WalkingEngineBase {
public:
  WalkingEngine();

private:
  JointRequest jointRequest; /**< The target joint request is provided by the walk and stand update methods. */
  Pose2D speed;              /**< The request for the UNSW walk generator. */
  Pose2D target;             /**< The relative walk target in target mode. */
  WalkGeneratorData::WalkMode walkMode = WalkGeneratorData::speedMode; /**< How is the request interpreted? */
  WalkRequest walkRequest;                                             /**< The currently executed walkRequest. */

  unsigned lastTimeWalking = 0; /**< The last time the engine was walking in ms. */

  void update(WalkingEngineOutput& walkingEngineOutput) override;

  /**
   * Fills the walking engine output from the values computed by the UNSW walk generator.
   * @param stepStarted A new step has just begun.
   * @param walkingEngineOutput The output of this module that is filled.
   */
  void updateOutput(bool stepStarted, WalkingEngineOutput& walkingEngineOutput);
};
