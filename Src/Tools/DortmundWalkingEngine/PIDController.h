/**
 * @file PIDController.h
 *
 * A class for controlling the speed of angular motors
 * combining three different controllers.
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#pragma once

class PIDController2 {
private:
  /** The constant factor for a proportional control part (gain) */
  double pControlFactor;
  /** The constant factor for an integral control part (reset) */
  double iControlFactor;
  /** The constant factor for a derivative control part (rate) */
  double dControlFactor;
  /** The time step length of a simulation control step*/
  double timeStep;
  /** The sum of errorValues */
  double errorSum;
  /** The error the previous pass */
  double previousError;

public:
  /** Constructor */
  PIDController2();

  /**
   * Returns the controller output.
   * @param measuredValue The current value of the joint
   * @param setpoint point The desired value of the joint
   * @return The controller output
   */
  double getControllerOutput(double measuredValue, double setpoint);

  /**
   * Sets the time step length of a simulation control step
   * @param value the time step length
   */
  void setTimeStep(double value) { timeStep = value; };

  /**
   * Updates the all control factors
   * @param p The new p value
   * @param i The new p value
   * @param d The new p value
   */
  void updateControlFactors(double p, double i, double d) {
    pControlFactor = p;
    iControlFactor = i;
    dControlFactor = d;
  }
};
