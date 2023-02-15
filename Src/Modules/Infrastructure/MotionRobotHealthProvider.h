/**
 * @file MotionRobotHealthProvider.h
 *
 * This file implements a module that provides information about the robot's health.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Core/Module/Module.h"
#include "Core/RingBufferWithSum.h"
#include "Representations/Infrastructure/RobotHealth.h"

MODULE(MotionRobotHealthProvider)
PROVIDES(MotionRobotHealth)
END_MODULE

/**
 * @class MotionRobotHealthProvider
 * A module that provides information about the robot's health
 */
class MotionRobotHealthProvider : public MotionRobotHealthProviderBase {
private:
  /** The main function, called every cycle
   * @param motionRobotHealth The data struct to be filled
   */
  void update(MotionRobotHealth& motionRobotHealth);

  RingBufferWithSum<unsigned, 30> timeBuffer; /** Buffered timestamps of previous executions */
  unsigned lastExecutionTime;

public:
  /** Constructor. */
  MotionRobotHealthProvider() : lastExecutionTime(0) {}
};
