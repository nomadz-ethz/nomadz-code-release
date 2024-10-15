#include "nomadz_audio_processing/audio_provider.hpp"

#include <cassert>
#include <chrono>
#include <algorithm>
#include <random>

#ifdef TARGET_ROBOT
#include <pwd.h>
#include <alsa/asoundlib.h>
#else
#endif

#include <rclcpp/rclcpp.hpp>

namespace nomadz_audio_processing {

  AudioDataProvider::AudioDataProvider() {
#ifdef TARGET_ROBOT
    handle_ = nullptr;

    unsigned i;
    for (i = 0; i < DEFAULT_SETTINGS_.retries; ++i) {
      if (snd_pcm_open(&handle_, "hw:0", SND_PCM_STREAM_CAPTURE, 0) >= 0)
        break;
      RCLCPP_WARN(rclcpp::get_logger("audio_provider"), "Failed to open audio device, retry %u", i);
      rclcpp::sleep_for(std::chrono::milliseconds(DEFAULT_SETTINGS_.retry_delay));
    }
    assert(i < DEFAULT_SETTINGS_.retries);

    snd_pcm_hw_params_t* params = nullptr;
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(handle_, params);
    snd_pcm_hw_params_set_access(handle_, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(handle_, params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_rate_near(handle_, params, &DEFAULT_SETTINGS_.sample_rate, 0);
    snd_pcm_hw_params_set_channels(handle_, params, DEFAULT_SETTINGS_.channels);
    snd_pcm_hw_params(handle_, params);
    snd_pcm_prepare(handle_);

    RCLCPP_INFO(rclcpp::get_logger("audio_provider"), "Audio device initialized successfully");
    audio_sample_.working_driver_0 = true;
#else
    audio_sample_.working_driver_0 = true;
#endif
  }

#ifdef TARGET_ROBOT
  AudioDataProvider::~AudioDataProvider() {
    if (handle_) {
      snd_pcm_close(handle_);
    }
  }
#endif

  AudioDataProvider::AudioSample AudioDataProvider::updateData() {
    audio_sample_.channels = DEFAULT_SETTINGS_.channels;
    audio_sample_.sample_rate = DEFAULT_SETTINGS_.sample_rate;

#ifdef TARGET_ROBOT
    if (!handle_ || !audio_sample_.working_driver_0) {
      RCLCPP_ERROR(rclcpp::get_logger("audio_provider"), "Audio device not properly initialized");
      return audio_sample_;
    }

    snd_pcm_state_t state = snd_pcm_state(handle_);

    if (state != SND_PCM_STATE_RUNNING && state != SND_PCM_STATE_PREPARED) {
      int err = snd_pcm_prepare(handle_);
      if (err < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("audio_provider"), "Cannot prepare audio interface: %s", snd_strerror(err));
        return audio_sample_;
      }
    }
    snd_pcm_sframes_t frames_to_read = DEFAULT_SETTINGS_.max_frames;
    audio_sample_.samples.resize(frames_to_read * audio_sample_.channels);

    snd_pcm_sframes_t frames_read = snd_pcm_readi(handle_, audio_sample_.samples.data(), frames_to_read);
    if (frames_read < 0) {
      snd_pcm_recover(handle_, frames_read, 0);
      assert(snd_pcm_readi(handle_, audio_sample_.samples.data(), frames_to_read) < 0);
      audio_sample_.samples.clear();
    } else {
      audio_sample_.samples.resize(frames_read * audio_sample_.channels);
    }
    return audio_sample_;
#else
    unsigned int size = DEFAULT_SETTINGS_.max_frames;
    audio_sample_.samples = std::vector<short>(size * audio_sample_.channels, 0);
    return audio_sample_;
#endif
  }

} // namespace nomadz_audio_processing
