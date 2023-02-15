/**
 * @file GyroTiltController.h
 *
 * This file implements a simple body orientation controller.
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <A href="mailto:Stefan.Czarnetzki@tu-dortmund.de">Stefan Czarnetzki</A>
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/MotionControl/BodyTilt.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/Sensing/InertiaSensorData.h"

MODULE(GyroTiltController)
REQUIRES(InertiaSensorData)
REQUIRES(WalkingEngineParams)
REQUIRES(Footpositions)
USES(WalkingInfo)
PROVIDES_WITH_MODIFY(BodyTilt)
DEFINES_PARAMETER(float, gyroFilterReactivity, 0.01f)
DEFINES_PARAMETER(float, gyroDriftCompensationFilterReactivity, 0.001f)
DEFINES_PARAMETER(float, gyroOffsetX, 0.f)
DEFINES_PARAMETER(float, gyroOffsetY, 0.f)
DEFINES_PARAMETER(float, orientationFilterReactivity, 0.05f)
END_MODULE

/**
 * @class GyroTiltController
 * Determines the target orientation of the body.
 */
class GyroTiltController : public GyroTiltControllerBase {
public:
  /** Constructor with all needed source data structures.
   * @param theSensorData Measured data.
   * @param theWalkingEngineParams Walking Engine Parameters.
   */
  GyroTiltController();

  /** Destructor */
  ~GyroTiltController();

  /**
   * Calculates the next target orientation.
   * @param bodyTilt Target data structure.
   */
  void update(BodyTilt& bodyTilt);

  void init();

private:
  bool initialized;
  float gyroX, gyroY, oriX, oriY, IX, IY; // , eX, eY;

  void filterGyroValues();
};
