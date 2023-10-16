/**
 * @file FootSupportProvider.h
 *
 * This file declares a module that provides an abstract distribution of
 * how much each foot supports the weight of the robot.
 *
 * The code is based on parts of the class BodyModel from the code of the
 * team UNSW Australia.
 *
 * This file is subject to the terms of the BHuman 2017 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Thomas Röfer
 */

#pragma once

#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Core/Module/Module.h"

MODULE(FootSupportProvider)

REQUIRES(FrameInfo)
REQUIRES(SensorData)
PROVIDES_WITH_MODIFY_AND_OUTPUT(FootSupport)
DEFINES_PARAMETER(float, minPressure, (0.1f))              /**< Minimum pressure assumed. */
DEFINES_PARAMETER(float, maxPressure, (5.0f))              /**< Maximum pressure assumed. */
DEFINES_PARAMETER(float, outerWeight, (0.8f))              /**< Weights for outer FSRs. */
DEFINES_PARAMETER(float, innerWeight, (0.3f))              /**< Weights for inner FSRs. */
DEFINES_PARAMETER(int, highestPressureUpdateTime, (10000)) /**< Update the highestPressure after so much time is past. */
DEFINES_PARAMETER(float,
                  thresholdHighChangeWeight,
                  (0.5f)) /**< Current support foot weight minimum to detect a support foot switch if the change is fast. */
DEFINES_PARAMETER(float,
                  thresholdLowChangeWeight,
                  (0.45f)) /**< Current support foot weight minimum to detect a support foot switch if the change is slow. */
DEFINES_PARAMETER(float, thresholdHighChangeVel, (0.3f))    /**< Definition of a high support foot weight velocity. */
DEFINES_PARAMETER(float, thresholdLowChangeVel, (0.15f))    /**< Definition of a slow support foot weight velocity. */
DEFINES_PARAMETER(float, resetMinMaxCheckThreshold, (0.5f)) /**< Only do a prediction, if the weight on the support foot was
                                                               at least this high since the last support switch. */

END_MODULE

class FootSupportProvider : public FootSupportProviderBase {
  ENUM(FsrSensorPosition, fl, fr, bl, br);

  float weights[2][FsrSensorPosition::numOfFsrSensorPositions];            /**< Weights for the individual FSRs. */
  float highestPressure[2][FsrSensorPosition::numOfFsrSensorPositions];    /**< Highest pressure measured so far per FSR. */
  float newHighestPressure[2][FsrSensorPosition::numOfFsrSensorPositions]; /**< Highest pressure measured in the last
                                                                              <highestPressureUpdateTime>/1000 seconds per
                                                                              FSR. */
  unsigned int updatePressureTimestamp;                                    /** Timestamp of last highest pressure update. */
  float lastSupport;                                                       /** Last support value. */
  float lastLastSupport;                                                   /** Second last support value. */
  bool wasOverMaxThreshold; /** Was support value above a min positive value. */
  bool wasOverMinThreshold; /** Was support value above a min negative value. */

  void update(FootSupport& theFootSupport) override;

public:
  FootSupportProvider();
};
