/**
 * @file AudioProvider.h
 *
 * This file implements a module that provides audio samples.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Thomas Röfer
 */
#pragma once

#ifdef TARGET_ROBOT
#include "alsa/asoundlib.h"
#endif
#include "Core/Module/Module.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/GameInfo.h"

MODULE(AudioProvider)
REQUIRES(GameInfo)
PROVIDES_WITH_OUTPUT(AudioData)
DEFINES_PARAMETER(bool, calibrateBothCameras, true)
DEFINES_PARAMETER(unsigned, retries, 10)     /**< Number of tries to open device. */
DEFINES_PARAMETER(unsigned, retryDelay, 500) /**< Delay before a retry to open device. */
DEFINES_PARAMETER(unsigned, channels, 2)     /**< Number of channels to capture. */
DEFINES_PARAMETER(unsigned,
                  sampleRate,
                  8000) /**< Sample rate to capture. This variable will contain the framerate the driver finally selected. */
DEFINES_PARAMETER(unsigned, maxFrames, 5000) /**< Maximum number of frames read in one cycle. */
END_MODULE

class AudioProvider : public AudioProviderBase {
private:
#ifdef TARGET_ROBOT
  snd_pcm_t* handle;
#endif
  void update(AudioData& audioData);

public:
  /**
   * Default constructor.
   */
  AudioProvider();

  /**
   * Destructor.
   */
  ~AudioProvider();
};
