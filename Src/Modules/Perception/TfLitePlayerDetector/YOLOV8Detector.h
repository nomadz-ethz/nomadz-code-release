/**
 * @file YOLOV8Detector.h
 *
 * Inference wrapper for a YOLO V8 object detection model.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <memory>
#include <string>
#include <utility>

#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/model.h>
#include <tensorflow/lite/signature_runner.h>

#include "Representations/Infrastructure/Image.h"

class YOLOV8Detector {
public:
  using BoundingBox = std::array<int, 4>;

  YOLOV8Detector(const std::string& pathToModel);

  std::vector<std::pair<BoundingBox, float>> detect(const Image& image, float minScore = 0.5f);

private:
  /** Number of threads used by the TFLite interpreter 2 works best on the NAO V6
   */
  static constexpr int NUM_INTERPRETER_THREADS = 2;

  std::unique_ptr<tflite::FlatBufferModel> model_;
  std::unique_ptr<tflite::Interpreter> interpreter_;

  tflite::SignatureRunner* detectSignatureRunner_;

  size_t inputWidth_;
  size_t inputHeight_;
};
