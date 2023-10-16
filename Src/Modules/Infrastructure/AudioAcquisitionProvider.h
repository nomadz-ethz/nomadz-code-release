/**
 * @file AudioAcquisitionProvider.h
 *
 * This file declares a module that provides audio recordings.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Core/Debugging/Debugging.h"
#include "Core/RingBuffer.h"
#include "Core/Settings.h"
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <string>
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/AudioAcquisition.h"

MODULE(AudioAcquisitionProvider)
REQUIRES(AudioData)
PROVIDES(AudioAcquisition)
END_MODULE

const int AUDIO_BUFF_LEN = 8000 * 2;
const int AUDIO_BUFF_THRESH = 2000 * 2;

class AudioAcquisitionProvider : public AudioAcquisitionProviderBase {
private:
  RingBuffer<short, AUDIO_BUFF_LEN> inputAudio; /** Audio data from the first channel */
  unsigned count;

  void update(AudioAcquisition& audioAcquisition);
  void recordNewAudio();

public:
  AudioAcquisitionProvider();
};
