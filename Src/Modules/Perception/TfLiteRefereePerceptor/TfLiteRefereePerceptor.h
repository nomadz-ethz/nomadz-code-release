/**
 * @file TfLiteRefereePerceptor.h
 *
 * Loads a model to classify balls using TFLite
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"       // Core instead of Tools
#include "Core/Debugging/Debugging.h" // Core instead of Tools
#include "Representations/Perception/RefereePercept.h"

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Whistle.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
#include <queue>
#include <deque>
#include <math.h>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/model.h"
//#include "tensorflow/lite/delegates/flex/delegate.h" // only for res3d_clstm model

MODULE(TfLiteRefereePerceptor)
REQUIRES(Image)
REQUIRES(FrameInfo)
REQUIRES(CameraInfo)
REQUIRES(ImageCoordinateSystem)
REQUIRES(CameraMatrix)
REQUIRES(FieldDimensions)
REQUIRES(RobotPose)
USES(Whistle)

LOADS_PARAMETER(std::string, pathToModel)
LOADS_PARAMETER(int, img_height)
LOADS_PARAMETER(int, img_width)
LOADS_PARAMETER(int, features_size)
LOADS_PARAMETER(int, total_frames)
LOADS_PARAMETER(float, confidence_threshold)
PROVIDES_WITH_MODIFY(RefereePercept)
END_MODULE

class TfLiteRefereePerceptor : public TfLiteRefereePerceptorBase {

public:
  /**
   * Loads the network and configures the parameters for inference
   * i.e. input shape and class labels.
   */
  TfLiteRefereePerceptor();

  ~TfLiteRefereePerceptor() = default;

  void update(RefereePercept& refereePercept);

private:
  // Model parameters

  // Input tensor for network, load image data into this tensor
  float* input_tensor;

  // Intermediate tensor
  float* intermediate_tensor;

  // Feature queue
  std::deque<std::shared_ptr<float>> feature_queue;

  void load_frame(const Image& current_image);
  void predict(RefereePercept& refereePercept, const std::deque<std::shared_ptr<float>>& queue);
  void majority_voting(RefereePercept& refereePercept);
  float position_weight(RefereePercept& refereePercept);

  // Model and Interpreter of TfLite
  std::unique_ptr<tflite::FlatBufferModel> model_1_; // feature extractor
  std::unique_ptr<tflite::FlatBufferModel> model_2_; // remaining model
  std::unique_ptr<tflite::Interpreter> interpreter_1_;
  std::unique_ptr<tflite::Interpreter> interpreter_2_;

  // Softmax of specific signal in output layer
  float softmax(int signal, float* output) {
    float sum = 0;
    for (int i = 0; i < 13; ++i) {
      sum += expf(output[i]);
    }
    return expf(output[signal]) / sum;
  }
};
