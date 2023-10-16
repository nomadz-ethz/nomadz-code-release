#include "YOLOV8Detector.h"

#include <tensorflow/lite/signature_runner.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/optional_debug_tools.h>
#include <tensorflow/lite/c/common.h>
#include <tensorflow/lite/kernels/internal/tensor_ctypes.h>

#include "Core/System/BHAssert.h"

YOLOV8Detector::YOLOV8Detector(const std::string& pathToModel) {
  // Load flatbuffer model
  model_ = tflite::FlatBufferModel::BuildFromFile(pathToModel.c_str());
  ASSERT(model_ != nullptr);

  // Configure the interpreter
  tflite::ops::builtin::BuiltinOpResolver resolver;
  tflite::InterpreterBuilder(*model_, resolver)(&interpreter_);
  ASSERT(interpreter_ != nullptr);

  // Set number of threads
  interpreter_->SetNumThreads(NUM_INTERPRETER_THREADS);

  // Get "detect" signature runner
  detectSignatureRunner_ = interpreter_->GetSignatureRunner("detect");

  // Allocate tensors
  detectSignatureRunner_->AllocateTensors();

  inputHeight_ = detectSignatureRunner_->input_tensor("image")->dims->data[0];
  inputWidth_ = detectSignatureRunner_->input_tensor("image")->dims->data[1];
  ASSERT(detectSignatureRunner_->input_tensor("image")->dims->data[2] == 1);
}

std::vector<std::pair<YOLOV8Detector::BoundingBox, float>> YOLOV8Detector::detect(const Image& image, float minScore) {

  TfLiteTensor* inputTensor = detectSignatureRunner_->input_tensor("image");
  float* rawInputPtr = tflite::GetTensorData<float>(inputTensor);

  const float widthRatio = static_cast<float>(image.width) / static_cast<float>(inputWidth_);
  const float heightRatio = static_cast<float>(image.height) / static_cast<float>(inputHeight_);

  auto toInputImageBbox = [&](float y1, float x1, float y2, float x2) -> BoundingBox {
    return {std::max(0, static_cast<int>(x1 * widthRatio)),
            std::max(0, static_cast<int>(y1 * heightRatio)),
            std::min(static_cast<int>(x2 * widthRatio), image.width),
            std::min(static_cast<int>(y2 * heightRatio), image.height)};
  };

  // Resize the image
  Image resizedImage = image.resizeAndFillUpInterLinear(inputWidth_, inputHeight_);
  for (size_t y = 0; y < inputHeight_; ++y) {
    for (size_t x = 0; x < inputWidth_; ++x) {
      rawInputPtr[y * inputWidth_ + x] = static_cast<float>(resizedImage[y][x].y);
    }
  }

  // Run inference
  detectSignatureRunner_->Invoke();

  // Parse output
  const TfLiteTensor* boxesTensor = detectSignatureRunner_->output_tensor("boxes");
  const float* boxesPtr = tflite::GetTensorData<float>(boxesTensor);
  const TfLiteTensor* scoresTensor = detectSignatureRunner_->output_tensor("scores");
  const float* scoresPtr = tflite::GetTensorData<float>(scoresTensor);
  const TfLiteTensor* numDetectionsTensor = detectSignatureRunner_->output_tensor("num_detections");
  const int maxNumDetections = tflite::GetTensorData<int>(numDetectionsTensor)[0];

  std::vector<std::pair<BoundingBox, float>> boundingBoxesWithScores;
  for (int i = 0; i < maxNumDetections; ++i) {
    if (scoresPtr[i] < minScore) {
      continue;
    }
    const float* boxPtr = &boxesPtr[i * 4];
    boundingBoxesWithScores.emplace_back(toInputImageBbox(boxPtr[0], boxPtr[1], boxPtr[2], boxPtr[3]), scoresPtr[i]);
  }

  return boundingBoxesWithScores;
}