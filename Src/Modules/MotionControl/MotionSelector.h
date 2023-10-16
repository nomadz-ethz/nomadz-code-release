/**
 * @file MotionSelector.h
 *
 * This file declares a module that is responsible for controlling the motion.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>, <A
 * href="mailto:allli@tzi.de">Alexander Härtl</A> and Jesse Richter-Klug
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/FallDownState.h"

MODULE(MotionSelector)
USES(SpecialActionsOutput)
USES(WalkingEngineOutput)
USES(KickEngineOutput)
REQUIRES(RobotInfo)
REQUIRES(FrameInfo)
REQUIRES(MotionRequest)
REQUIRES(GroundContactState)
REQUIRES(FallDownState)
PROVIDES_WITH_MODIFY(MotionSelection)
REQUIRES(MotionSelection)
LOADS_PARAMETER(bool, enableFallProtection)
END_MODULE

class MotionSelector : public MotionSelectorBase {
private:
  static PROCESS_WIDE_STORAGE(MotionSelector) theInstance; /**< The only instance of this module. */

  bool forceStand;
  MotionRequest::Motion lastMotion;
  MotionRequest::Motion prevMotion;
  unsigned lastExecution; // FIXME: Value is used uninitialized
  SpecialActionRequest::SpecialActionID lastActiveSpecialAction;

  void update(MotionSelection& motionSelection);

  void interpolate(float* ratios, const int amount, const int interpolationTime, const int targetMotion);

public:
  /**
   * Can be used to overwrite all other motion requests with a stand request.
   * Must be called again in every frame a stand is desired.
   */
  static void stand();
  static void move();

  MotionSelector()
      : lastMotion(MotionRequest::specialAction), prevMotion(MotionRequest::specialAction),
        lastActiveSpecialAction(SpecialActionRequest::playDead) {
    theInstance = this;
  }
};
