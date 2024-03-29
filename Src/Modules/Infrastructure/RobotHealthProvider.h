/**
 * @file RobotHealthProvider.h
 *
 * This file implements a module that provides information about the robot's health.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Core/Module/Module.h"
#include "Core/RingBufferWithSum.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/LineAnalysis.h"
#include "Representations/Perception/GoalPercept.h"
#ifdef TARGET_ROBOT
#include "Nao/NaoBody.h"
#endif

MODULE(RobotHealthProvider)
REQUIRES(BallPercept)
REQUIRES(LineAnalysis)
REQUIRES(GoalPercept)
REQUIRES(MotionRobotHealth)
REQUIRES(FilteredSensorData)
REQUIRES(FrameInfo)
LOADS_PARAMETER(char, batteryLow)     /**< The voltage below which the robot gives low battery warnings. */
LOADS_PARAMETER(int, temperatureHigh) /**< The temperature the robot starts complaining about the temperature. */
PROVIDES_WITH_MODIFY_AND_DRAW(RobotHealth)
LOADS_PARAMETER(bool, enableName)
END_MODULE

/**
 * @class RobotHealthProvider
 * A module that provides information about the robot's health
 */
class RobotHealthProvider : public RobotHealthProviderBase {
public:
  /** Constructor. */
  RobotHealthProvider();

private:
  RingBufferWithSum<unsigned, 30> timeBuffer; /**< Buffered timestamps of previous executions */
  unsigned lastExecutionTime;
  unsigned lastRelaxedHealthComputation;
  unsigned startBatteryLow; /**< Last time the battery state was not low. */
  float lastBatteryLevel;
  bool batteryVoltageFalling;
  unsigned highTemperatureSince;
#ifdef TARGET_ROBOT
  NaoBody naoBody;
  unsigned int lastBodyTemperatureReadTime;
  unsigned int lastWlanCheckedTime;
#endif

  /** The main function, called every cycle
   * @param robotHealth The data struct to be filled
   */
  void update(RobotHealth& robotHealth);
};
