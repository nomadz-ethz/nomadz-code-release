/**
 * @file XBoxJoystick.h
 *
 * Declares a joystick interface class for XBox 360 Controller (Linux implementation)
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

// Button enum
enum BUTTONS {
  A_BUTTON = 0,
  B_BUTTON = 1,
  X_BUTTON = 2,
  Y_BUTTON = 3,
  LB_BUTTON = 4,
  RB_BUTTON = 5,
  BACK_BUTTON = 6,
  START_BUTTON = 7,
  CENTER_BUTTON = 8, // With xbox symbol
  LW_BUTTON = 9,     // Left "steer wheel"
  RW_BUTTON = 10,    // Right "steer wheel"
};

// Axes enum
enum AXES {
  LWH_AXIS = 0, // Left "steer wheel" horizontally
  LWV_AXIS = 1, // Left "steer wheel" vertically
  RWH_AXIS = 2, // Right "steer wheel" horizontally
  RWV_AXIS = 3, // Right "steer wheel" vertically
  RT_AXIS = 4,
  LT_AXIS = 5,
  CH_AXIS = 6, //"Steer cross" horizontally
  CV_AXIS = 7, //"Steer cross" vertically
};

enum numElements {
  numOfAxes = 8,     /**< Number of supported axes. */
  numOfButtons = 11, /**< Number of supported buttons. */
};

/**
 * A joystick interface class.
 */
class XBoxJoystick {
private:
  double axisMaxVal = 32767.f;

public:
  XBoxJoystick();

  ~XBoxJoystick();

  bool init(int ID);

  bool checkConnection();

  bool getNextEvent(unsigned int& buttonId, bool& buttonPressed, unsigned int& axisID, double& axisVal);

  double getAxisState(unsigned int axisId) const;

  bool isButtonPressed(unsigned int buttonId) const;

private:
  int jd;
  unsigned int joystickId;
  double axisState[numOfAxes];
  unsigned int buttonState[2];
};
