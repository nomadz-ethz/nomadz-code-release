/**
 * @file FootSupportProvider.cpp
 *
 * This file implements a module that provides an abstract distribution of
 * how much each foot supports the weight of the robot.
 *
 * The code is based on parts of the class BodyModel from the code of the
 * team UNSW Australia.
 *
 * This file is subject to the terms of the BHuman 2017 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Thomas Röfer
 */

#include "FootSupportProvider.h"

MAKE_MODULE(FootSupportProvider, Sensing)

FootSupportProvider::FootSupportProvider() {
  weights[Legs::left][FsrSensorPosition::fl] = weights[Legs::left][FsrSensorPosition::bl] = outerWeight;
  weights[Legs::right][FsrSensorPosition::fr] = weights[Legs::right][FsrSensorPosition::br] = -outerWeight;
  weights[Legs::left][FsrSensorPosition::fr] = weights[Legs::left][FsrSensorPosition::br] = innerWeight;
  weights[Legs::right][FsrSensorPosition::fl] = weights[Legs::right][FsrSensorPosition::bl] = -innerWeight;

  for (int leg = 0; leg < Legs::numOfLegs; leg++) {
    for (int sensor = 0; sensor < numOfFsrSensorPositions; sensor++) {
      highestPressure[leg][sensor] = minPressure;
      newHighestPressure[leg][sensor] = minPressure;
    }
  }
  updatePressureTimestamp = 0;
  wasOverMaxThreshold = false;
  wasOverMinThreshold = false;
}

void FootSupportProvider::update(FootSupport& theFootSupport) {
  float totalPressure = 0.f;
  float weightedSum = 0.f;

  // switched is true, when it was set so in last motion frame
  if (theFrameInfo.getTimeSince(updatePressureTimestamp) > highestPressureUpdateTime && theFootSupport.switched) {
    updatePressureTimestamp = theFrameInfo.time;

    for (int leg = 0; leg < Legs::numOfLegs; leg++) {
      for (int sensor = 0; sensor < numOfFsrSensorPositions; sensor++) {
        highestPressure[leg][sensor] = newHighestPressure[leg][sensor];
        newHighestPressure[leg][sensor] = minPressure;
      }
    }
  }

  for (int leg = 0; leg < Legs::numOfLegs; leg++) {
    for (int sensor = 0; sensor < numOfFsrSensorPositions; sensor++) {
      float pressure = std::min(maxPressure, theSensorData.data[SensorData::fsrLFL + 4 * leg + sensor]);
      highestPressure[leg][sensor] = std::max(highestPressure[leg][sensor], pressure);
      newHighestPressure[leg][sensor] = std::max(newHighestPressure[leg][sensor], pressure);
      pressure /= highestPressure[leg][sensor];
      totalPressure += pressure;
      weightedSum += weights[leg][sensor] * pressure;
    }
  }

  if (std::abs(totalPressure) > 0.f) {
    lastLastSupport = lastSupport;
    lastSupport = theFootSupport.support;
    float prevSupport = theFootSupport.support;
    theFootSupport.support = weightedSum / totalPressure;
    theFootSupport.switched = prevSupport * theFootSupport.support < 0.f;

    float a = theFootSupport.support - lastSupport; // current vel
    float b = lastSupport - lastLastSupport;        // last vel

    // Checks to make sure, that the robot switched once the support foot and had enought weight on one foot
    // Filteres most false positives
    if (theFootSupport.support > resetMinMaxCheckThreshold) {
      wasOverMaxThreshold = true;
    } else if (theFootSupport.support < -resetMinMaxCheckThreshold) {
      wasOverMinThreshold = true;
    }
    if (theFootSupport.support < 0.f) {
      wasOverMaxThreshold = false;
    }
    if (theFootSupport.support > 0.f) {
      wasOverMinThreshold = false;
    }

    //(Was enought weight on the support foot) AND (is currently low weight on the foot) AND (is the weight changing fast
    // enough to make a "good" prediction)
    theFootSupport.predictedSwitched =
      ((wasOverMinThreshold &&
        ((theFootSupport.support > -thresholdHighChangeWeight && (a + b) > thresholdHighChangeVel) ||
         (theFootSupport.support > -thresholdLowChangeWeight && (a + b) > thresholdLowChangeVel)) &&
        a > b * 0.7f) ||
       (wasOverMaxThreshold &&
        ((theFootSupport.support < thresholdHighChangeWeight && (a + b) < -thresholdHighChangeVel) ||
         (theFootSupport.support < thresholdLowChangeWeight && (a + b) < -thresholdLowChangeVel)) &&
        a < b * 0.7f));
  } else {
    theFootSupport.support = 0.f;
    theFootSupport.switched = false;
    theFootSupport.predictedSwitched = false;
  }
}
