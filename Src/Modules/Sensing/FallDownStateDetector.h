/**
 * @file FallDownStateDetector.h
 *
 * This file declares a module that provides information about the current state of the robot's body.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:maring@informatik.uni-bremen.de">Martin Ring</a>
 */

#pragma once

#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Core/Module/Module.h"
#include "Core/RingBufferWithSum.h"

MODULE(FallDownStateDetector)
REQUIRES(SensorData)
REQUIRES(InertiaSensorData)
USES(MotionInfo)
REQUIRES(FrameInfo)
PROVIDES_WITH_MODIFY_AND_DRAW(FallDownState)
LOADS_PARAMETER(int, fallTime) /**< The time (in ms) to remain in state 'falling' after a detected fall */
LOADS_PARAMETER(
  float, staggeringAngleX) /**< The threshold angle which is used to detect the robot is staggering to the back or front*/
LOADS_PARAMETER(float, fallDownBuffer)

LOADS_PARAMETER(float, staggeringAngleY_front)
LOADS_PARAMETER(float,
                staggeringAngleY_back) /**< The threshold angle which is used to detect the robot is staggering sidewards*/
// LOADS_PARAMETER(float, fallDownAngleY)   /**< The threshold angle which is used to detect a fall to the back or front*/
LOADS_PARAMETER(float, fallDownAngleX) /**< The threshold angle which is used to detect a sidewards fall */
LOADS_PARAMETER(float, onGroundAngle)  /**< The threshold angle which is used to detect the robot lying on the ground */
END_MODULE

/**
 * @class FallDownStateDetector
 *
 * A module for computing the current body state from sensor data
 */
class FallDownStateDetector : public FallDownStateDetectorBase {
private:
  /** Executes this module
   * @param fallDownState The data structure that is filled by this module
   */
  void update(FallDownState& fallDownState);

  bool isFalling();
  bool isStaggering();
  bool isCalibrated();
  bool specialSpecialAction();
  bool isUprightOrStaggering(FallDownState& fallDownState);
  FallDownState::Direction directionOf(float angleX, float angleY);
  FallDownState::Sidestate sidewardsOf(FallDownState::Direction dir);

  unsigned lastFallDetected;

  /** Indices for buffers of sensor data */
  ENUM(BufferEntry, accX, accY, accZ);

  /** Buffers for averaging sensor data */
  RingBufferWithSum<float, 15> buffers[numOfBufferEntrys];

public:
  /**
   * Default constructor.
   */
  FallDownStateDetector();
};
