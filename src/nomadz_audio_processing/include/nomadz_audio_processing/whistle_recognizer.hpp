#pragma once

#include <vector>
#include <filesystem>

#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/model.h>

#include "nomadz_audio_processing/ring_buffer.hpp"

namespace nomadz_audio_processing {

  class WhistleRecognizer {
  public:
    struct Settings {
      const int WHISTLE_BUFF_LEN = 8000;
      const int WHISTLE_FRAME_LEN = 1000;
      const int WHISTLE_NUM_FRAMES = 8;
      const int WHISTLE_NUM_MFCC = 64;
      const int WHISTLE_OVERLAP = WHISTLE_NUM_FRAMES * 3 / 4;
    };

    static const Settings DEFAULT_SETTINGS;

    static const int BUFFER_LEN = 8000;

    struct Whistle {
      short confidence_of_last_whistle_detection = 0;
      unsigned int last_time_whistle_detected = 0;
      float volume = 0;
      bool whistle_detected = false;
      double wp_0 = 0.F;
      double wp_1 = 0.F;
      int buffer_size = 0;
      unsigned int sound_last_time_heard = 0;
    };
    int componentCount = 0;
    explicit WhistleRecognizer(const std::filesystem::path& model_path);

    bool
    recognize(const RingBuffer<short, BUFFER_LEN>& whistle_buffer, double& prob, Whistle& whistle_info, int channel_num);

    bool update(const std::vector<short>& audio_data);

  private:
    float calculateWhistleVolume();
    float input_scale_;
    float output_scale_;
    int input_zero_point_;
    int output_zero_point_;

    float whistle_threshold_ = 0.5;
    float volume_threshold_ = 0.007;
    std::vector<double> whistle_input_8khz_;
    std::vector<double> whistle_mfccs_;

    std::unique_ptr<tflite::FlatBufferModel> model_;
    std::unique_ptr<tflite::Interpreter> interpreter_;
    RingBuffer<short, BUFFER_LEN> whistle_buffer1_;
    RingBuffer<short, BUFFER_LEN> whistle_buffer2_;

    struct Whistle whistle_info_;
  };

} // namespace nomadz_audio_processing
