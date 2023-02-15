/**
 * @file OdometerProvider.h
 *
 * Declaration of module that computes some additional odometry information
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a> and marcel
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Modeling/Odometer.h"

MODULE(OdometerProvider)
REQUIRES(OdometryData)
PROVIDES_WITH_MODIFY(Odometer)
END_MODULE

/*
 * @class OdometerProvider
 *
 * Module that computes some additional odometry information
 */
class OdometerProvider : public OdometerProviderBase {
private:
  Pose2D lastOdometryData; /**< Odometry data in last frame */

  /**
   * The method that computes the odometry information
   *
   * @param odometer The odometry information that is updated by this module.
   */
  void update(Odometer& odometer);
};
