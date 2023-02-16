/**
 * @file PIDController.cpp
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#include "PIDController.h"

PIDController2::PIDController2()
    : pControlFactor(1.0), iControlFactor(1.0), dControlFactor(1.0), timeStep(1.0), errorSum(0.0), previousError(0.0) {}

double PIDController2::getControllerOutput(double measuredValue, double setpoint) {
  double error = setpoint - measuredValue;
  double pControlValue = pControlFactor * error;
  errorSum += error;
  double iControlValue = iControlFactor * timeStep * errorSum;
  double dControlValue = (dControlFactor * (error - previousError)) / timeStep;
  previousError = error;
  return pControlValue + iControlValue + dControlValue;
}
