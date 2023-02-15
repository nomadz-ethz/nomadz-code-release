/**
 * @file CSConverterModule.cpp
 *
 * This file implements a module that is a wrapper for the UNSW walk generator.
 *
 * This file is subject to the terms of the BHuman 2017 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
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
REQUIRES(WalkGeneratorData)
REQUIRES(PlannedSteps)
REQUIRES(WalkingEngineOutput)
REQUIRES(OrientationData)
PROVIDES_WITH_MODIFY(WalkingEngineOutput)
DEFINES_PARAMETER(int, minTimeInStandBeforeLeaving, (250)) /**< The minimum time in stand before leaving is possible. */
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
  WalkGeneratorData::WalkMode walkMode = WalkGeneratorData::speedMode; /**< How is the request interpreted? */
  WalkRequest walkRequest;                                             /**< The currently executed walkRequest. */

  std::function<Pose3D(float)>
    getKickFootOffset; /**< If set, provides an offset to add to the pose of the swing foot to create a kick motion. */

  Pose2D target;                /**< The relative walk target in target mode. */
  Pose2D lastTarget;            /**< The last request processed. */
  unsigned lastTimeWalking = 0; /**< The last time the engine was walking in ms. */

  void update(WalkingEngineOutput& walkingEngineOutput) override;

  /**
   * Fills the walking engine output from the values computed by the UNSW walk generator.
   * @param stepStarted A new step has just begun.
   * @param walkingEngineOutput The output of this module that is filled.
   */
  void updateOutput(bool stepStarted, WalkingEngineOutput& walkingEngineOutput);

  /**
   * Fills in the walking speeds in the request for the Walk2014Generator.
   * @param speed The speed or step size to walk with.
   * @param walkMode Speed or step size?
   * @param getKickFootOffset If set, provides an offset to add to the pose of the swing foot to
   *                          create a kick motion.
   */
  void walk(const Pose2D& speed,
            WalkGeneratorData::WalkMode walkMode = WalkGeneratorData::speedMode,
            const std::function<Pose3D(float)>& getKickFootOffset = std::function<Pose3D(float)>());

  /**
   * Fills in the walking target in the request for the Walk2014Generator.
   * @param speed The speed to walk with. If everything is zero, the robot stands.
   * @param target The target to walk to.
   */
  void walk(const Pose2D& speed, const Pose2D& target);

  /**
   * Sets the request for the Walk2014Generator to make the robot stand.
   */
  void stand();

  /**
   * The method updates the walk request, but keeps the walk kick request
   * inside it.
   */
  void updateWalkRequestWithoutKick();

/**
 * Is the left leg swinging during a pre-step for kicking?
 * The symbol is a macro to always reflect the current state of the walkKickRequest.
 */
#define leftPreStepPhase (walkRequest.walkKickRequest.kickLeg == Legs::right)
};
