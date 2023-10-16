/**
 * @file DeepFieldBoundaryProvider.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */
#include "DeepFieldBoundaryProvider.h"
#include "Core/System/File.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"
#include "Core/Debugging/DebugDrawings.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/optional_debug_tools.h"

#include <opencv2/opencv.hpp>
#include <string>

MAKE_MODULE(DeepFieldBoundaryProvider, Perception)

DeepFieldBoundaryProvider::DeepFieldBoundaryProvider() {
  // Load the FlatBuffer model

  auto absolutePath = std::string(File::getBHDir()) + "/" + pathToModel;
  model_ = tflite::FlatBufferModel::BuildFromFile(absolutePath.c_str());
  ASSERT(model_ != nullptr);
  // Configure the intepreter
  tflite::ops::builtin::BuiltinOpResolver resolver;
  tflite::InterpreterBuilder(*model_, resolver)(&interpreter_);
  interpreter_->AllocateTensors();
}

void DeepFieldBoundaryProvider::preprocessAndLoadInput(float* input) {

  // Convert the YCbCr channels of the image to a cv mat

  auto const input_height = interpreter_->input_tensor(0)->dims->data[1];
  auto const input_width = interpreter_->input_tensor(0)->dims->data[2];

  Image rescaled_input = theImage.resizeAndFillUpInterLinear(input_width, input_height);

  // Load in the input tensor
  for (int x = 0; x < input_width; x++) {
    for (int y = 0; y < input_height; y++) {
      input[y * input_width * 3 + x * 3 + 0] = rescaled_input[y][x].y;
      input[y * input_width * 3 + x * 3 + 1] = rescaled_input[y][x].cr;
      input[y * input_width * 3 + x * 3 + 2] = rescaled_input[y][x].cb;
    }
  }
}

void DeepFieldBoundaryProvider::update(FieldBoundary& fieldBoundary) {
  if (theRefereePercept.runModel) {
    return;
  }
  
  // Preprocess the input image and load it into the input tensor
  DECLARE_DEBUG_DRAWING("module:DeepFieldBoundaryProvider:boundaries", "drawingOnImage");
  float* input = interpreter_->typed_input_tensor<float>(0);
  fieldBoundary.boundarySpots.clear();
  fieldBoundary.model.clear();
  if (theCameraInfo.camera == CameraInfo::lower) {
    return;
  }
  preprocessAndLoadInput(input);

  // Run the inference
  interpreter_->Invoke();

  float* output = interpreter_->typed_output_tensor<float>(0);

  std::vector<float> output_vector;
  float delta = 1.0 / (interpreter_->output_tensor(0)->dims->data[1] - 1);
  for (int i = 0; i < interpreter_->output_tensor(0)->dims->data[1]; i++) {
    output_vector.push_back(delta * i);
  }
  std::vector<float> output_vec;
  for (int i = 0; i < interpreter_->output_tensor(0)->dims->data[1]; i++) {
    output_vec.push_back(output[i]);
  }
  std::vector<float> best_model = findBestModel(output_vector, output_vec);
  for (int i = 0; i < 4; i++) {
    fieldBoundary.model.push_back(best_model[i]);
  }
  std::vector<float> y = getColumnWiseY(best_model, theImage.width);
  for (int x = 0; x < theImage.width; x++) {
    Vector2<int> point(x, theImage.height * (y[x]) - 1);
    fieldBoundary.boundarySpots.push_back(point);
  }
  displayInferenceResults(best_model);
}

void DeepFieldBoundaryProvider::displayInferenceResults(const std::vector<float>& model) {
  // Display the inference results
  std::vector<float> y = getColumnWiseY(model, theImage.width);
  for (int x = 0; x < theImage.width; x++) {
    MID_DOT("module:DeepFieldBoundaryProvider:boundaries",
            x,
            theImage.height * (y[x]) - 1,
            ColorRGBA(255, 0, 0),
            ColorRGBA(0, 0, 255));
  }
}

std::vector<float> DeepFieldBoundaryProvider::fitLine(const std::vector<float>& x, const std::vector<float>& y) {
  cv::Mat A = cv::Mat::zeros(x.size(), 2, CV_32F);
  cv::Mat B = cv::Mat::zeros(x.size(), 1, CV_32F);
  for (int i = 0; i < x.size(); i++) {
    A.at<float>(i, 0) = 1;
    A.at<float>(i, 1) = x[i];
    B.at<float>(i, 0) = y[i];
  }
  cv::Mat X = cv::Mat::zeros(2, 1, CV_32F);
  cv::solve(A, B, X, cv::DECOMP_SVD);
  std::vector<float> result;
  float res = 0;
  for (int i = 0; i < x.size(); i++) {
    res += (y[i] - (X.at<float>(0, 0) + X.at<float>(1, 0) * x[i])) * (y[i] - (X.at<float>(0, 0) + X.at<float>(1, 0) * x[i]));
  }
  result.push_back(X.at<float>(0, 0));
  result.push_back(-1);
  result.push_back(-1);
  result.push_back(X.at<float>(1, 0) + X.at<float>(0, 0));
  result.push_back(res);
  return result;
}

// find the best model
std::vector<float> DeepFieldBoundaryProvider::findBestModel(const std::vector<float>& x, const std::vector<float>& y) {
  std::vector<float> best_model = fitLine(x, y);
  std::vector<float> left_model, right_model;
  float x_inter, y_inter;

  for (int i = 2; i < x.size() - 2; i++) {
    left_model = fitLine(std::vector<float>(x.begin(), x.begin() + i), std::vector<float>(y.begin(), y.begin() + i));
    right_model = fitLine(std::vector<float>(x.begin() + i, x.end()), std::vector<float>(y.begin() + i, y.end()));
    if (left_model[4] + right_model[4] < best_model[4]) {
      x_inter = (left_model[0] - right_model[0]) / (right_model[3] - left_model[3] - right_model[0] + left_model[0]);
      y_inter = left_model[0] + (left_model[3] - left_model[0]) * x_inter;
      if (x_inter < 0 or x_inter > 1) {
        continue;
      }
      if (y_inter > std::max(left_model[0], right_model[3])) {
        continue;
      }
      best_model[0] = left_model[0];
      best_model[1] = x_inter;
      best_model[2] = y_inter;
      best_model[3] = right_model[3];
      best_model[4] = left_model[4] + right_model[4];
    }
  }
  return std::vector<float>(best_model.begin(), best_model.begin() + 4);
}

float DeepFieldBoundaryProvider::getFieldBoundaryY(const std::vector<float>& model, float x) {
  if (model[1] == -1) {
    return std::max(0.0f, std::min(1.0f, model[0] + (model[3] - model[0]) * x));
  }
  if (x < model[1]) {
    return std::max(0.0f, std::min(1.0f, model[0] + (model[2] - model[0]) * x / model[1]));
  } else {
    return std::max(0.0f, std::min(1.0f, model[2] + (model[3] - model[2]) * (x - model[1]) / (1 - model[1])));
  }
}

std::vector<float> DeepFieldBoundaryProvider::getColumnWiseY(const std::vector<float>& model, int x) {
  std::vector<float> y;
  for (int i = 0; i < x; i++) {
    y.push_back(1.0f * getFieldBoundaryY(model, (0.5 + i) / x));
  }
  return y;
}
