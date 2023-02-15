/**
 * @file XBoxJoystick.cpp
 *
 * Implementation of the joystick interface class for XBox 360 Controller (Linux implementation)
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cerrno>
#include <linux/joystick.h>
#include <unistd.h>
#include <cstdio>
#include <iostream>
#include <assert.h>

#include "XBoxJoystick.h"
//#include "Core/System/BHAssert.h"

XBoxJoystick::XBoxJoystick() : jd(-1) {}

XBoxJoystick::~XBoxJoystick() {
  if (jd != -1) {
    close(jd);
  }
}

bool XBoxJoystick::init(int ID) {

  // ASSERT(jd == -1);
  assert(jd == -1);

  // Define "path" to controller
  char devname[16];
  snprintf(devname, 16, "/dev/input/js%i", ID);

  // Try to open device
  jd = open(devname, O_RDONLY | O_NONBLOCK);
  if (jd != -1) { // if connection is here, init parametes
    joystickId = ID;
    buttonState[0] = buttonState[1] = 0;
    for (int i = 0; i < numOfAxes; ++i) {
      axisState[i] = 0;
    }
    return true;
  }

  return false;
}

bool XBoxJoystick::checkConnection() {
  if (jd == -1) {
    return false;
  }

  return true;
}

bool XBoxJoystick::getNextEvent(unsigned int& buttonId, bool& buttonPressed, unsigned int& axisID, double& axisVal) {
  // ASSERT(jd != -1);
  // assert(jd == -1);
  struct js_event e;
  ssize_t r;
  while ((r = read(jd, &e, sizeof(struct js_event))) > 0) {

    // button press or release event
    if (e.type & JS_EVENT_BUTTON && e.number < numOfButtons) {
      buttonId = e.number;
      buttonPressed = e.value ? true : false;
      if (buttonPressed) {
        buttonState[e.number / 32] |= 1 << (buttonId % 32);
      } else {
        buttonState[e.number / 32] &= ~(1 << (buttonId % 32));
      }
      return true;
    }
    // axis position changed
    else if (e.type & JS_EVENT_AXIS && e.number < numOfAxes) {
      // Shift value for 4th and 5th axis
      double val = (double)e.value;

      if (e.number == 4 || e.number == 5) {
        double tempVal = val;
        val = (tempVal + axisMaxVal) / 2;
      }

      axisState[e.number] = val / axisMaxVal;
      axisID = (unsigned int)e.number;
      axisVal = axisState[e.number];
      return true;
    }
  }

  return false;
}

double XBoxJoystick::getAxisState(unsigned int axisId) const {
  // ASSERT(jd != -1);
  // assert(jd == -1);
  // ASSERT(axisId < numOfAxes);
  assert(axisId < numOfAxes);
  return axisState[axisId];
}

bool XBoxJoystick::isButtonPressed(unsigned int buttonId) const {
  // ASSERT(jd != -1);
  // assert(jd == -1);
  // ASSERT(buttonId < numOfButtons);
  assert(buttonId < numOfButtons);
  return buttonState[buttonId / 32] & (1 << (buttonId % 32)) ? true : false;
}
