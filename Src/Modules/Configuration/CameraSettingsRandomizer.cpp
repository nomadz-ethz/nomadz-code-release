/**
 * @file CameraSettingsRandomizer.cpp
 *
 * Randomizes camera settings every once in a while
 * Use this to generate diverse lighting conditions for your training dataset.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "CameraSettingsRandomizer.h"

void CameraSettingsRandomizer::update(CameraSettings& cameraSettings) {
  ++updatesSinceLastChange;

  if (updatesSinceLastChange >= cycleDuration) {
    updatesSinceLastChange = 0;

    std::uniform_int_distribution<int> contrastDistribution(contrastRange[0], contrastRange[1]);
    cameraSettings.contrast.value = contrastDistribution(generator);

    std::uniform_int_distribution<int> saturationDistribution(saturationRange[0], saturationRange[1]);
    cameraSettings.saturation.value = saturationDistribution(generator);

    std::uniform_int_distribution<int> hueDistribution(hueRange[0], hueRange[1]);
    cameraSettings.hue.value = hueDistribution(generator);

    std::uniform_int_distribution<int> exposureDistribution(exposureRange[0], exposureRange[1]);
    cameraSettings.exposure.value = exposureDistribution(generator);

    std::uniform_int_distribution<int> gainDistribution(gainRange[0], gainRange[1]);
    cameraSettings.gain.value = gainDistribution(generator);
  }
}

MAKE_MODULE(CameraSettingsRandomizer, Cognition Infrastructure)
