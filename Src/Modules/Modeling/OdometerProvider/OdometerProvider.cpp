/**
 * @file OdometerProvider.cpp
 *
 * Implementation of module that computes some additional odometry information
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a> and marcel
 */

#include "OdometerProvider.h"

MAKE_MODULE(OdometerProvider, Modeling)

void OdometerProvider::update(Odometer& odometer) {
  odometer.odometryOffset = theOdometryData - lastOdometryData;
  odometer.distanceWalked += odometer.odometryOffset.translation.abs();
  lastOdometryData = theOdometryData;
}
