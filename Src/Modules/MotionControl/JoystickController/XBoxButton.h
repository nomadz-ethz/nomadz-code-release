/**
 * @file XBoxButton.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <vector>

class XBoxButton {
private:
  bool status;
  bool executeAction;
  unsigned int blockCounter;
  unsigned int maxBlockCounter;

public:
  XBoxButton();
  ~XBoxButton();

  void init(unsigned int frameFrequency);
  bool update(bool buttonPressed, unsigned int frameFrequency);
  bool getStatus();

  static float getAxisCommand(float axisVal, const std::vector<float> bounds, bool mirrorAxis);

private:
  void setStatus();
};
