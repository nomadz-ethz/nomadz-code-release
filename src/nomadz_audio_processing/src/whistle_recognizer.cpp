#include "nomadz_audio_processing/whistle_recognizer.hpp"

#include <cmath>

#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/tools/gen_op_registration.h>

#include "nomadz_audio_processing/libmfcc.hpp"

namespace nomadz_audio_processing {

  const WhistleRecognizer::Settings WhistleRecognizer::DEFAULT_SETTINGS;

  WhistleRecognizer::WhistleRecognizer(const std::filesystem::path& model_path) {
    model_ = tflite::FlatBufferModel::BuildFromFile(model_path.c_str());
    if (!model_) {
      throw std::runtime_error("Failed to load model");
    }
    assert(model_ != nullptr);
    // Configure the interpreter
    tflite::ops::builtin::BuiltinOpResolver resolver;
    tflite::InterpreterBuilder(*model_, resolver)(&interpreter_);
    if (!interpreter_) {
      throw std::runtime_error("Failed to construct interpreter");
    }
    interpreter_->AllocateTensors();
    for (int i = 0; i < DEFAULT_SETTINGS.WHISTLE_BUFF_LEN; i++) {
      whistle_buffer1_.add(0);
      whistle_buffer2_.add(0);
    }

    whistle_input_8khz_.resize(DEFAULT_SETTINGS.WHISTLE_BUFF_LEN);
    whistle_mfccs_.resize(static_cast<std::vector<double>::size_type>(DEFAULT_SETTINGS.WHISTLE_NUM_FRAMES) *
                          static_cast<std::vector<double>::size_type>(DEFAULT_SETTINGS.WHISTLE_NUM_MFCC));

    auto* input = interpreter_->input_tensor(0);
    auto* output = interpreter_->output_tensor(0);

    input_scale_ = input->params.scale;
    input_zero_point_ = input->params.zero_point;
    output_scale_ = output->params.scale;
    output_zero_point_ = output->params.zero_point;
  }

  bool WhistleRecognizer::update(const std::vector<short>& audio_data) {

    const unsigned int time_till_clear = 1000;
    whistle_info_.volume = -0.1F;
    if (whistle_info_.whistle_detected) {
      // TODO (Arka/ Giuliano): Add a method to get the current time from the frame
      if (whistle_info_.last_time_whistle_detected + 2000 - whistle_info_.last_time_whistle_detected >= time_till_clear) {
        whistle_info_.whistle_detected = false;
      } else {
        return false;
      }
    }
    whistle_info_.whistle_detected = false;

    // TODO (Axel/ Arka): Add a method to get the current number of channels from the audioprovider
    // if (whistle.channel_input != 1) { Throw an error}

    // TODO (Axel/ Arka): Add a method to get the current audio data from the audioprovider
    // if the size of audiodata is zero, return
    unsigned int i = 0;
    while (i < audio_data.size()) {
      componentCount++;
      whistle_info_.buffer_size++;
      whistle_buffer1_.add(audio_data[i++]);
      whistle_buffer2_.add(audio_data[i++]);
    }

    double prob_channel0 = 0;
    double prob_channel1 = 0;
    float current_volume = 0;

    if (whistle_buffer1_.full() && whistle_info_.buffer_size >= DEFAULT_SETTINGS.WHISTLE_OVERLAP) {
      componentCount = 0;
      whistle_info_.buffer_size = 0;
      current_volume = calculateWhistleVolume();
      whistle_info_.volume = current_volume;
      if (current_volume < volume_threshold_) {
        return false;
      }

      const bool w_0 = recognize(whistle_buffer1_, prob_channel0, whistle_info_, 0);
      const bool w_1 = recognize(whistle_buffer2_, prob_channel1, whistle_info_, 1);
      double max_prob = std::max(prob_channel0, prob_channel1);

      if (max_prob < 0.5) { // Whistle is not detected
        return false;
      }
      if (w_0 && w_1) { // TODO (Arka): This considers that both the drivers are working
        whistle_info_.whistle_detected = true;
        whistle_info_.last_time_whistle_detected = 2; // TODO (Arka): Add a method to get the current time from the frame
        whistle_info_.confidence_of_last_whistle_detection = static_cast<short>(max_prob * 100);
        whistle_info_.wp_0 = prob_channel0;
        whistle_info_.wp_1 = prob_channel1;
        whistle_info_.confidence_of_last_whistle_detection = 100;
      }
      if (!w_0) {
        whistle_info_.wp_0 = 0;
        whistle_info_.wp_1 = prob_channel1;
        whistle_info_.confidence_of_last_whistle_detection = 50;
      }
      if (!w_1) {
        whistle_info_.wp_0 = prob_channel0;
        whistle_info_.wp_1 = 0;
        whistle_info_.confidence_of_last_whistle_detection = 50;
      }

      whistle_info_.whistle_detected = static_cast<bool>(whistle_info_.confidence_of_last_whistle_detection > 0);
      return whistle_info_.whistle_detected;
    }
    return false;
  }

  bool WhistleRecognizer::recognize(const RingBuffer<int16_t, BUFFER_LEN>& whistle_buffer,
                                    double& prob,
                                    Whistle& whistle_info,
                                    int channel_num) {

    for (int i = 0; i < DEFAULT_SETTINGS.WHISTLE_BUFF_LEN; i++) {
      whistle_input_8khz_[i] = static_cast<double>(whistle_buffer[i]);
    }

    for (int j = 0; j < DEFAULT_SETTINGS.WHISTLE_NUM_FRAMES; j++) {
      std::vector<double> mfcc_result(DEFAULT_SETTINGS.WHISTLE_NUM_MFCC);
      std::vector<double> data(whistle_input_8khz_.begin() + static_cast<ptrdiff_t>(j) * DEFAULT_SETTINGS.WHISTLE_FRAME_LEN,
                               whistle_input_8khz_.begin() +
                                 static_cast<ptrdiff_t>(j + 1) * DEFAULT_SETTINGS.WHISTLE_FRAME_LEN);

      mfcc(mfcc_result,
           data,
           /* sampling rate */ 8192,
           /* filters */ 80,
           DEFAULT_SETTINGS.WHISTLE_NUM_MFCC,
           DEFAULT_SETTINGS.WHISTLE_FRAME_LEN,
           /* lifter */ 0,
           /* append energy? */ 0,
           /* window */ 2,
           DEFAULT_SETTINGS.WHISTLE_FRAME_LEN);

      std::copy(mfcc_result.begin(),
                mfcc_result.end(),
                whistle_mfccs_.begin() + static_cast<ptrdiff_t>(j) * DEFAULT_SETTINGS.WHISTLE_NUM_MFCC);
    }

    auto* input = interpreter_->typed_input_tensor<uint8_t>(0);
    for (int j = 0; j < DEFAULT_SETTINGS.WHISTLE_NUM_FRAMES * DEFAULT_SETTINGS.WHISTLE_NUM_MFCC; j++) {
      input[j] = static_cast<uint8_t>((whistle_mfccs_[j] / input_scale_ + input_zero_point_));
    }
    if (interpreter_->Invoke() != kTfLiteOk) {
      throw std::runtime_error("Failed to invoke interpreter");
    }
    auto* output = interpreter_->typed_output_tensor<uint8_t>(0);

    prob = 1.0 - (static_cast<double>(output[0] - output_zero_point_)) * output_scale_;
    if (channel_num == 0) {
      whistle_info.wp_0 = prob;
    } else {
      whistle_info.wp_1 = prob;
    }

    return prob > whistle_threshold_;
  }

  float WhistleRecognizer::calculateWhistleVolume() {
    // Current version considers both the drivers are working. Needs to be changed in future
    //  TODO(Arka): Add conditions if any of the drivers fail
    float volume0 = 0.0F;
    float volume1 = 0.0F;

    for (int i = 0; i < whistle_buffer1_.getNumberOfEntries(); i++) {
      float sample =
        static_cast<float>(whistle_buffer1_.getEntry(i)) / static_cast<float>(std::numeric_limits<short>::max());
      volume0 += std::abs(sample);
    }
    for (int i = 0; i < whistle_buffer2_.getNumberOfEntries(); i++) {
      float sample =
        static_cast<float>(whistle_buffer2_.getEntry(i)) / static_cast<float>(std::numeric_limits<short>::max());
      volume1 += std::abs(sample);
    }

    volume0 = volume0 / static_cast<float>(whistle_buffer1_.getNumberOfEntries());
    volume1 = volume1 / static_cast<float>(whistle_buffer2_.getNumberOfEntries());

    return (volume0 + volume1) / 2;
  }

} // namespace nomadz_audio_processing
