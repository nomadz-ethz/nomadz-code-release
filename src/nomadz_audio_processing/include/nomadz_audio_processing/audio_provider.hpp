#pragma once

#ifdef TARGET_ROBOT
#include <alsa/asoundlib.h>
#endif

#include <vector>

namespace nomadz_audio_processing {

  class AudioDataProvider {
  public:
    AudioDataProvider();
    struct AudioSample {
      std::vector<short> samples;
      unsigned int sample_rate = 48000;
      unsigned int channels = 2;
      bool working_driver_0 = false;
    };
    struct Settings {
      unsigned int retries = 10;
      int retry_delay = 500;
      int channels = 2;
      unsigned int sample_rate = 8000;
      unsigned max_frames = 5000;
      int buffer_size = 8000;
    };

    AudioSample updateData();
    // Declare the destructor
    ~AudioDataProvider();

  private:
    Settings DEFAULT_SETTINGS_;
    AudioSample audio_sample_;
#ifdef TARGET_ROBOT
    snd_pcm_t* handle_;
#endif
  };

} // namespace nomadz_audio_processing
