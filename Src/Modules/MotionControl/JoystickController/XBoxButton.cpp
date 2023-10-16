/**
 * @file XBoxButton.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include <iostream>

#include "XBoxButton.h"

XBoxButton::XBoxButton() {}
XBoxButton::~XBoxButton() {}

void XBoxButton::init(unsigned int frameFrequency) {
  status = false;
  executeAction = false;
  blockCounter = 0;
  maxBlockCounter = frameFrequency;
}

bool XBoxButton::update(bool buttonPressed, unsigned int frameFrequency) {
  if (buttonPressed) {
    if (blockCounter == 0) {
      executeAction = true;
      setStatus();
      blockCounter++;
    } else {
      executeAction = false;
      if (blockCounter < maxBlockCounter) {
        blockCounter++;
      } else {
        blockCounter = 0;
      }
    }
  } else {
    executeAction = false;
    blockCounter = 0;
  }
  maxBlockCounter = frameFrequency;
  return executeAction;
}

bool XBoxButton::getStatus() {
  return status;
}

void XBoxButton::setStatus() {
  if (status) {
    status = false;
  } else {
    status = true;
  }
}

float XBoxButton::getAxisCommand(float axisVal, const std::vector<float> bounds, bool mirrorAxis) {
  float minPos = bounds[0];
  float zeroPos = bounds[1];
  float maxPos = bounds[2];

  if (mirrorAxis) {
    axisVal *= (-1.f);
  }

  if (axisVal >= zeroPos) {
    return (zeroPos + axisVal * (maxPos - zeroPos));
  } else {
    return (zeroPos + axisVal * (zeroPos - minPos));
  }
}
