/**
 * @file TfLiteRefereePerceptor.cpp
 *
 * Implements module TfLiteRefereePerceptor.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include "TfLiteRefereePerceptor.h"

#include "Core/System/File.h" // used to be "Platform/Common/File.h"
#include "Representations/Infrastructure/Image.h"
#include "Core/Debugging/DebugDrawings.h" // Core instead of Tools

#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/optional_debug_tools.h"

MAKE_MODULE(TfLiteRefereePerceptor, Perception)

TfLiteRefereePerceptor::TfLiteRefereePerceptor() {
  // Load the model and initialize interpreter
  std::string referee_models_path = pathToModel;
  std::string feature_tflite = "feature_extractor.tflite";
  std::string remaining_tflite = "remaining_model.tflite";

  auto absolutePath = std::string(File::getBHDir()) + referee_models_path;
  model_1_ = tflite::FlatBufferModel::BuildFromFile((absolutePath + feature_tflite).c_str());
  model_2_ = tflite::FlatBufferModel::BuildFromFile((absolutePath + remaining_tflite).c_str());
  ASSERT(model_1_ != nullptr);
  ASSERT(model_2_ != nullptr);

  tflite::ops::builtin::BuiltinOpResolver resolver_1;
  tflite::ops::builtin::BuiltinOpResolver resolver_2;
  tflite::InterpreterBuilder builder_1(*model_1_, resolver_1);
  tflite::InterpreterBuilder builder_2(*model_2_, resolver_2);
  builder_1(&interpreter_1_);
  builder_2(&interpreter_2_);
  VERIFY(interpreter_1_->AllocateTensors() == kTfLiteOk);
  VERIFY(interpreter_2_->AllocateTensors() == kTfLiteOk);

  EXECUTE_ONLY_IN_DEBUG(tflite::PrintInterpreterState(interpreter_1_.get()););
  EXECUTE_ONLY_IN_DEBUG(tflite::PrintInterpreterState(interpreter_2_.get()););

  // Fetch input shape and check dimensions
  VERIFY(interpreter_1_->inputs().size() == 1 && interpreter_1_->outputs().size() == 1);
  VERIFY(interpreter_2_->inputs().size() == 1 && interpreter_2_->outputs().size() == 1);
  auto input_1 = interpreter_1_->input_tensor(0);
  auto output_1 = interpreter_1_->output_tensor(0);
  auto input_2 = interpreter_2_->input_tensor(0);
  auto output_2 = interpreter_2_->output_tensor(0);

  // Feature extractor input and output dimensions
  VERIFY(input_1->dims->size == 4 && input_1->dims->data[0] == 1 && input_1->dims->data[1] == img_height &&
         input_1->dims->data[2] == img_width && input_1->dims->data[3] == 3);
  VERIFY(output_1->dims->size == 2 && output_1->dims->data[0] == 1 && output_1->dims->data[1] == features_size);

  // Remaining model input and output dimensions
  VERIFY(input_2->dims->size == 3 && input_2->dims->data[0] == 1 && input_2->dims->data[1] == total_frames &&
         input_2->dims->data[2] == features_size);
  VERIFY(output_2->dims->size == 2 && output_2->dims->data[0] == 1 && output_2->dims->data[1] == 13);

  // Input tensor for the network: Load the image data into this tensor
  input_tensor = interpreter_1_->typed_input_tensor<float>(0);
  intermediate_tensor = interpreter_2_->typed_input_tensor<float>(0);
}

void TfLiteRefereePerceptor::load_frame(const Image& current_image) {
  Image resized_YCrCb = current_image.resizeAndFillUpInterLinear(img_width, img_height);
  Image resized_RGB;
  resized_RGB.convertFromYCbCrToRGB(resized_YCrCb);

  for (size_t y = 0; y < img_height; ++y) {
    for (size_t x = 0; x < img_width; ++x) {
      // Input tensor dimensions: (frames, height, width, channels:BGR)
      input_tensor[3 * img_width * y + 3 * x] = (float)(resized_RGB[y][x].b) / 255;
      input_tensor[3 * img_width * y + 3 * x + 1] = (float)(resized_RGB[y][x].g) / 255;
      input_tensor[3 * img_width * y + 3 * x + 2] = (float)(resized_RGB[y][x].r) / 255;
    }
  }
  return;
}

void TfLiteRefereePerceptor::predict(RefereePercept& refereePercept, const std::deque<std::shared_ptr<float>>& queue) {
  // Initialize intermediate tensor
  for (int i = 0; i < 15; ++i) {
    float* feature_ptr = queue[i].get();
    for (int j = 0; j < features_size; ++j) {
      intermediate_tensor[i * features_size + j] = feature_ptr[j];
    }
  }

  VERIFY(interpreter_2_->Invoke() == kTfLiteOk);
  intermediate_tensor = interpreter_2_->typed_input_tensor<float>(0);
  float* output_tensor = interpreter_2_->typed_output_tensor<float>(0);

  // Find argmax signal (default: no signal)
  int signal = 0;
  for (int i = 1; i < 13; ++i) {
    if (output_tensor[i] > output_tensor[signal]) {
      signal = i; // new most likely signal
    }
  }

  // Confidence of argmax signal in %
  float confidence = softmax(signal, output_tensor);

  OUTPUT_TEXT("Predicted signal: " << signal);
  OUTPUT_TEXT("Confidence:" << confidence);

  // Initialize prediction vector
  if (refereePercept.preds.empty()) {
    for (int i = 0; i < 13; ++i) {
      refereePercept.preds.push_back(0);
    }
  }

  // Update prediction vector
  if (confidence >= confidence_threshold) {
      refereePercept.preds[signal] += 1;
  }

  return;
}

void TfLiteRefereePerceptor::majority_voting(RefereePercept& refereePercept) {
  // Initialize weights for signals, based on which prediction is made
  // Important: Player exchange singal has two phases, which is why it's weight is high
  std::vector<int> weights = {1 /*no-signal*/, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 20 /*full-time*/, 50 /*player-exchange*/};
  int overall_signal = 0;
  int total_sum = 0;
  for (int i = 0; i < 13; ++i) {
    if (refereePercept.preds[i] * weights[i] > refereePercept.preds[overall_signal] * weights[overall_signal]) {
      overall_signal = i;
    }
    total_sum += refereePercept.preds[i] * weights[i];
  }
  int scores = refereePercept.preds[overall_signal] * weights[overall_signal];
  if (overall_signal == 12) {
    // Player exchange singal (first phase)
    // Determine which side the referee arm is extended (second phase)
    int even_preds = 0;
    int odd_preds = 0;
    for (int i = 1; i < 11; ++i) {
      if (i % 2 == 0) {
        even_preds += refereePercept.preds[i] * weights[i];
      } else {
        odd_preds += refereePercept.preds[i] * weights[i];
      }
    }
    overall_signal = (odd_preds >= even_preds) ? 12 : 13;
  }
  refereePercept.signal = overall_signal;
  if (total_sum) {
    refereePercept.predScore = scores * 1.0f / (total_sum);
  } else {
    refereePercept.predScore = 0;
    refereePercept.signal = 0;
  }
  return;
}

void TfLiteRefereePerceptor::update(RefereePercept& refereePercept) {
  if (refereePercept.gamecontrollerSignal != -1) {
    if (theFrameInfo.time - refereePercept.lastTimeGameControllerSent >= 5000) {
      refereePercept.gamecontrollerSignal = -1;
      refereePercept.signal = -1;
    }
  }

  if (!refereePercept.runModel) {
    refereePercept.signal = -1;
    refereePercept.preds.clear();
    // clear feature queue
    feature_queue.clear();
    return;
  }

  if (feature_queue.size() < 15) {
    // read current Image only if it is from the upper camera
    if (theCameraInfo.camera == theCameraInfo.upper) {
      Image current_image = theImage;
      // load frame data into input tensor of network
      load_frame(current_image);

      VERIFY(interpreter_1_->Invoke() == kTfLiteOk);
      float* output_tensor_ptr = interpreter_1_->typed_output_tensor<float>(0);
      std::shared_ptr<float> feature(new float[features_size], std::default_delete<float[]>());
      std::memcpy(feature.get(), output_tensor_ptr, features_size * sizeof(float));
      feature_queue.push_back(feature);
    }
  }

  else {
    predict(refereePercept, feature_queue);
    majority_voting(refereePercept);
    // remove first five elements of queue
    for (int i = 0; i < 5; ++i) {
      feature_queue.pop_front();
    }
  }

  return;
}

float TfLiteRefereePerceptor::position_weight(RefereePercept& refereePercept) {
  std::vector<std::vector<float>> weights = {{11, 12, 13}, {21, 22, 23}, {31, 32, 33}};
  float third_x = (2 * theFieldDimensions.xPosOpponentGoalPost) / 3;
  float third_y = (2 * theFieldDimensions.yPosLeftSideline) / 3;
  float adapted_pos_x;
  float adapted_pos_y;

  adapted_pos_x = theFieldDimensions.xPosOpponentGoalPost + theRobotPose.translation.x;

  adapted_pos_y = theFieldDimensions.yPosLeftSideline + theRobotPose.translation.y;

  int area_x = adapted_pos_x / third_x;
  int area_y = adapted_pos_y / third_y;
  return weights[area_x][area_y];
}
