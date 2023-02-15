/**
 * @file TfLiteRefereePerceptor.h
 *
 * Loads a model to classify balls using TFLite
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
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
USES(RobotPose)
USES(Whistle)

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
  // Input tensor for network, load image data into this tensor
  float* input_tensor;

  // Confidence threshold to pass prediction to representation, see state machine for behaviour:
  // React directly if confidence over 90% otherwise react to most likely signal over 10 sec period
  int const conf_threshold = 65;

  // Store last prediction for signal 11 (only react if twice in a row)
  int last_pred;

  void load_frame(Image current_image, int nr_frames);
  void predict(RefereePercept& refereePercept);

  // Model and Interpreter of TfLite
  std::unique_ptr<tflite::FlatBufferModel> model_;
  std::unique_ptr<tflite::Interpreter> interpreter_;

  // number of frames currently loaded into input tensor
  int nr_frames;
};
