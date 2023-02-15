/**
 * @file TfLiteRefereePerceptor.cpp
 *
 * Implements module TfLiteRefereePerceptor.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
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
  std::string model_path = "/Config/Models/Base/Referee/model_0714_tl2_15f.tflite"; // careful if 10 or 15 frames

  auto absolutePath = std::string(File::getBHDir()) + model_path;
  model_ = tflite::FlatBufferModel::BuildFromFile(absolutePath.c_str());
  ASSERT(model_ != nullptr);

  tflite::ops::builtin::BuiltinOpResolver resolver;
  tflite::InterpreterBuilder builder(*model_, resolver);
  builder(&interpreter_);
  VERIFY(interpreter_->AllocateTensors() == kTfLiteOk);

  OUTPUT_TEXT("TfLiteRefereePerceptor: Loaded model and initialized interpreter!");
  EXECUTE_ONLY_IN_DEBUG(tflite::PrintInterpreterState(interpreter_.get()););

  // Fetch input shape and check dimensions
  VERIFY(interpreter_->inputs().size() == 1 && interpreter_->outputs().size() == 1);
  auto input = interpreter_->input_tensor(0);
  auto output = interpreter_->output_tensor(0);

  // Input dimensions: 10 or 15 frames, 112x112 image, 3 channels
  VERIFY(input->dims->size == 4 && input->dims->data[0] == 15 && input->dims->data[1] == 112 &&
         input->dims->data[2] == 112 && input->dims->data[3] == 3);

  // Outputs dimensions: Vector with probabilites for each of the 12 signals (including NO-signal)
  VERIFY(output->dims->size == 1 && output->dims->data[0] == 12);

  // Input tensor for the network: Load the image data into this tensor
  input_tensor = interpreter_->typed_input_tensor<float>(0);

  // Last predictions: 0 (NO-signal) at start
  last_pred = 0;

  nr_frames = 0;
}

void TfLiteRefereePerceptor::load_frame(Image current_image, int nr_frames) {
  // Convert image to CVMat in order to resize
  cv::Mat image = current_image.convertToCVMat();
  cv::Mat image_resized;
  cv::resize(image, image_resized, cv::Size(112, 112), cv::INTER_LINEAR);

  // Import back to "Image" file since that's how pixel data can be read for input tensor
  Image resized_YCrCb;
  resized_YCrCb.importFromCVMat(image_resized);

  // Function "importFromCVMat" uses YCrCb so convert to RGB
  Image resized_RGB;
  resized_RGB.convertFromYCbCrToRGB(resized_YCrCb);

  for (size_t y = 0; y < 112; ++y) {   // height
    for (size_t x = 0; x < 112; ++x) { // width
      // Load RGB data into network input tensor, normalized to [0,1]
      // Input tensor dimensions: (frames, height, width, channels:BGR)
      input_tensor[3 * 112 * 112 * nr_frames + 3 * 112 * y + 3 * x] = (float)(resized_RGB[y][x].b) / 255;
      input_tensor[3 * 112 * 112 * nr_frames + 3 * 112 * y + 3 * x + 1] = (float)(resized_RGB[y][x].g) / 255;
      input_tensor[3 * 112 * 112 * nr_frames + 3 * 112 * y + 3 * x + 2] = (float)(resized_RGB[y][x].r) / 255;
    }
  }
  return;
}

void TfLiteRefereePerceptor::predict(RefereePercept& refereePercept) {
  VERIFY(interpreter_->Invoke() == kTfLiteOk);

  // Network output: probabilities for 12 signals
  float* output_tensor = interpreter_->typed_output_tensor<float>(0);

  // Find argmax signal (default: no signal)
  int signal = 0;
  for (int i = 1; i < 12; ++i) {
    if (output_tensor[i] > output_tensor[signal]) {
      signal = i; // new most likely signal
    }
  }

  // Confidence of argmax signal
  int confidence = int(output_tensor[signal] * 100);

  // std::ofstream outfile;
  // outfile.open("output.txt", std::ios_base::app); // append instead of overwrite
  // outfile << "Predicted signal: " << signal << std::endl;
  // outfile << "Confidence %: " << confidence << std::endl;

  OUTPUT_TEXT("Predicted signal: " << signal);
  OUTPUT_TEXT("Confidence %:" << confidence);

  // If confidence % over threshold update representation
  if (confidence >= conf_threshold && signal != 0) {
    if (signal <= 6) {
      // Signals 1-6, react directly
      refereePercept.signal = signal;
      refereePercept.confidence = confidence;
    } else if (signal == last_pred) {
      // Signals 7-11, react only if predicted twice in a row
      refereePercept.signal = signal;
      refereePercept.confidence = confidence;
    }
    // update last prediction
    last_pred = signal;
  }

  return;
}

void TfLiteRefereePerceptor::update(RefereePercept& refereePercept) {
  // reset representation
  refereePercept.signal = -1;
  refereePercept.confidence = 0;

  if (nr_frames < 15) {
    // read current Image only if it is from the upper camera
    if (theCameraInfo.camera == theCameraInfo.upper) {
      Image current_image = theImage;

      // load frame data into input tensor of netwokr
      load_frame(current_image, nr_frames);

      // increase nr_frames counter (in RefereePercept Representation)
      ++nr_frames;
    }
  }

  if (nr_frames == 15) {
    // make prediction and reset nr_frames
    predict(refereePercept);
    nr_frames = 0;
  }

  return;
}
