/**
 * @file AudioAcquisitionProvider.cpp
 *
 * This file declares a module that provides audio recordings.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include "AudioAcquisitionProvider.h"
#include "Core/System/File.h"

AudioAcquisitionProvider::AudioAcquisitionProvider() {
  for (int i = 0; i < AUDIO_BUFF_LEN; ++i) {
    inputAudio.add(0);
  }
  count = 0;
}

void AudioAcquisitionProvider::update(AudioAcquisition& audioAcquisition) {
  unsigned int i = 0;
  while (i < theAudioData.samples.size()) {
    inputAudio.add(theAudioData.samples[i++]);
    count++;
  }

  if (count > AUDIO_BUFF_THRESH) {
    recordNewAudio();
    count = 0;
  }
}

void AudioAcquisitionProvider::recordNewAudio() {
  time_t now = time(0);
  struct tm* timeinfo;
  char buffer[80];

  timeinfo = localtime(&now);
  strftime(buffer, sizeof(buffer), "_%d%m%Y_%H%M%S", timeinfo);

  std::string str(buffer);

  OutBinaryFile file(std::string(File::getBHDir()) + "/rawAudio" + str + ".dat");
  for (int i = 0; i < AUDIO_BUFF_LEN; i++) {
    file << inputAudio[i];
  }
}

MAKE_MODULE(AudioAcquisitionProvider, Cognition Infrastructure)
