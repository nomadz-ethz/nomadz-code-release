/**
 * @file CameraSettingsRandomizer.h
 *
 * Randomizes camera settings every once in a while
 * Use this to generate diverse lighting conditions for your training dataset.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <random>
#include "Core/Module/Module.h"
#include "Representations/Configuration/CameraSettings.h"

MODULE(CameraSettingsRandomizer)
PROVIDES_WITH_MODIFY(CameraSettings)
LOADS_PARAMETER(unsigned int, cycleDuration) // Only randomize settings every "cycleDuration" updates
LOADS_PARAMETER(std::vector<int>, contrastRange)
LOADS_PARAMETER(std::vector<int>, saturationRange)
LOADS_PARAMETER(std::vector<int>, hueRange)
LOADS_PARAMETER(std::vector<int>, exposureRange)
LOADS_PARAMETER(std::vector<int>, gainRange)
END_MODULE

class CameraSettingsRandomizer : public CameraSettingsRandomizerBase {
public:
  CameraSettingsRandomizer() : updatesSinceLastChange(0) {}

  void update(CameraSettings& cameraSettings);

private:
  unsigned int updatesSinceLastChange;
  std::default_random_engine generator;
};
