/**
 * @file MotionRobotHealthProvider.cpp
 *
 * This file implements a module that provides information about the robot's health.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#include "MotionRobotHealthProvider.h"
#include "Core/System/Time.h"

void MotionRobotHealthProvider::update(MotionRobotHealth& motionRobotHealth) {
  // Compute frame rate of motion process:
  unsigned now = Time::getCurrentSystemTime();
  if (lastExecutionTime != 0) {
    timeBuffer.add(now - lastExecutionTime);
  }
  motionRobotHealth.motionFrameRate =
    timeBuffer.getSum() ? 1000.0f / (static_cast<float>(timeBuffer.getSum()) / timeBuffer.getNumberOfEntries()) : 0.0f;
  motionRobotHealth.avgMotionTime = float(timeBuffer.getAverage());
  motionRobotHealth.maxMotionTime = float(timeBuffer.getMaximum());
  motionRobotHealth.minMotionTime = float(timeBuffer.getMinimum());
  lastExecutionTime = now;
}

MAKE_MODULE(MotionRobotHealthProvider, Motion Infrastructure)
