/**
 * @file FootSteps
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
 */

#pragma once

#include <vector>

#include "Tools/DortmundWalkingEngine/StepData.h"
#include "Core/System/BHAssert.h"

#ifndef WALKING_SIMULATOR
#include "Core/Streams/Streamable.h"
#else
#include "bhumanstub.h"
#endif

/** Maximum number of possible foot steps in buffer */
#define MAX_STEPS 300

/**
 * @class Robot
 * Representation to transfer the target foot steps to other modules.
 */
STREAMABLE_DECLARE_LEGACY(FootSteps)
class FootSteps : public FootStepsBaseWrapper {
public:
  /** When the controller is not running this step is used. */
  StepData suggestedStep;

  /** Is the controller running? */
  bool running;

  /** Immediate stop in case of emergency. */
  bool emergencyStop;

  /** Target pitch. Obsolete. */
  double pitch;

  /** Position of the robot after the whole preview phase is executed */
  Point robotPoseAfterStep;

  /** Duration of one step, obsolete */
  int cycleLength;

  /** Position in this step, obsolete */
  int positionInCycle;

  /**
   * The provided time is the number of the frame when the currently created foot step
   * will be executed
   */
  unsigned time;

  void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(running)
    STREAM(pitch)
    STREAM(time)
    STREAM_REGISTER_FINISH;
  };

  /** Constructor */
  FootSteps() : steps(MAX_STEPS) {
    running = false;
    emergencyStop = false;
  };

  /** Initialize the data. */
  void reset() {
    running = false;
    emergencyStop = false;
    steps.clear();
  }

  /** Destructor */
  ~FootSteps(){};

  int getNumOfSteps() const { return static_cast<int>(steps.size()); }

  bool empty() const { return steps.empty(); }

  void addStep(Footposition newstep) { steps.push_back(newstep); }

  Footposition getStep(unsigned int i) const {
    ASSERT(!steps.empty());
    return steps[i];
  }

private:
  /**
   *	Buffer for target foot steps. There might be more than one step
   *	per frame to fill the preview buffer needed by the preview controller
   *	ZMP/IP-Controller
   */

  std::vector<Footposition> steps;
};
