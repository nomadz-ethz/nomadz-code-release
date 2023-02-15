/**
 * @file DeepPlayerPerceptor.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "DeepPlayerPerceptor.h"
#include "Core/System/File.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"

#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/optional_debug_tools.h"

#include <opencv2/opencv.hpp>
#include <string>

MAKE_MODULE(DeepPlayerPerceptor, Perception)

DeepPlayerPerceptor::DeepPlayerPerceptor() {
  // Load the FlatBuffer model
  auto absolutePath = std::string(File::getBHDir()) + "/" + pathToModel;
  model_ = tflite::FlatBufferModel::BuildFromFile(absolutePath.c_str());
  ASSERT(model_ != nullptr);

  // Configure the intepreter
  tflite::ops::builtin::BuiltinOpResolver resolver;
  tflite::InterpreterBuilder(*model_, resolver)(&interpreter_);
  VERIFY(interpreter_->AllocateTensors() == kTfLiteOk);

  // Configure the output tensor idxs
  for (size_t idx = 0; idx < interpreter_->outputs().size(); ++idx) {
    std::string output_name = interpreter_->GetOutputName(idx);
    // All output tensors in the default signature are named as
    // "StatefulPartition:<IDX>" and IDX identifies the type of output
    // the tensor holds i.e. num of detections, boxes, classes, scores
    switch (std::stoi(output_name.substr(output_name.length() - 1))) {
    case NUM_DETECTIONS:
      output_tensor_idxs[NUM_DETECTIONS] = idx;
      break;
    case SCORES:
      output_tensor_idxs[SCORES] = idx;
      break;
    case CLASSES:
      output_tensor_idxs[CLASSES] = idx;
      break;
    case BOXES:
      output_tensor_idxs[BOXES] = idx;
      break;
    default:
      assert(false);
    }
  }
}

void DeepPlayerPerceptor::preprocessAndLoadInput(float* input_tensor) {

  // Convert the Y channel of the image to a cv mat
  cv::Mat image_y = theImage.convertToCVMatYChan();

  // Resize the input image
  auto const input_height = interpreter_->input_tensor(0)->dims->data[1];
  auto const input_width = interpreter_->input_tensor(0)->dims->data[2];
  cv::Mat resized_image_y;
  cv::resize(image_y, resized_image_y, cv::Size(input_width, inputHeight), cv::INTER_LINEAR);

  // Rescale the input values to the range [-1, 1]
  cv::Mat rescaled_input;
  resized_image_y.convertTo(rescaled_input, CV_32F, kNormalizationAlpha, kNormalizationBeta);

  // Load in the input tensor
  for (size_t x = 0; x < input_width; ++x) {
    for (size_t y = 0; y < input_height; ++y) {
      input_tensor[y * input_width + x] = rescaled_input.at<float>(y, x);
    }
  }
}

DeepPlayerPerceptor::BoundingBox DeepPlayerPerceptor::computeDenormalizedBoundingBox(
  NormalizedBoundingBox const& normalized_bbox, int image_width, int image_height) {

  auto ymin = std::max(static_cast<int>(std::round(normalized_bbox[0] * image_height)), 0);
  auto xmin = std::max(static_cast<int>(std::round(normalized_bbox[1] * image_width)), 0);
  auto ymax = std::min(static_cast<int>(std::round(normalized_bbox[2] * image_height)), image_height);
  auto xmax = std::min(static_cast<int>(std::round(normalized_bbox[3] * image_width)), image_width);

  return {ymin, xmin, ymax, xmax};
}

DeepPlayerPerceptor::InferenceResult DeepPlayerPerceptor::extractInferenceResultFromOutputTensor() {

  InferenceResult inference_result;

  auto const num_detections =
    static_cast<size_t>(interpreter_->typed_output_tensor<float>(output_tensor_idxs[NUM_DETECTIONS])[0]);

  // Find the boxes whose score is higher than the minimum allowed value
  auto scores = interpreter_->typed_output_tensor<float>(output_tensor_idxs[SCORES]);
  auto boxes = interpreter_->typed_output_tensor<float>(output_tensor_idxs[BOXES]);
  for (size_t i = 0; i < num_detections; ++i) {
    if (scores[i] > minimumDetectionScore) {
      // Fetch the corresponding bounding box
      NormalizedBoundingBox normalized_bbox;
      for (size_t j = 0, start = i * 4; j < 4; ++j) {
        normalized_bbox[j] = boxes[start + j];
      }
      // Add the denormalized bounding box and the corresponding score to the result
      inference_result.emplace_back(computeDenormalizedBoundingBox(normalized_bbox, theImage.width, theImage.height),
                                    scores[i]);
    }
  }

  return inference_result;
}

void DeepPlayerPerceptor::runJerseyScan(PlayerPercept::Player& player) {
  ColorClasses::Color opponentPlotColor = ColorClasses::red;

  int centerPosImageX = int(player.x2 - (player.x2 - player.x1) / 2);

  float ownJerseyCounter = 0;
  float numOfPx = 0;

  int scanIterY = player.jerseyY1;
  while (scanIterY < player.jerseyY0) {

    const Image::Pixel* px = &theImage[scanIterY][centerPosImageX];
    int blueValue = (int)px->b;
    int redValue = (int)px->r;
    numOfPx++;

    if (((theOwnTeamInfo.teamColor == TEAM_BLUE) && (blueValue >= blueJerseyRGBThreshold)) ||
        ((theOwnTeamInfo.teamColor == TEAM_RED) && (redValue >= redJerseyRGBThreshold))) {
      ownJerseyCounter++;
    }

    scanIterY++;
  }

  float ownJerseyRatio = 0.f;
  if (numOfPx > 0) {
    ownJerseyRatio = ownJerseyCounter / numOfPx;
  }

  if (ownJerseyRatio > ownJerseyThreshold) {
    player.detectedJersey = true;
    player.opponent = false;
    opponentPlotColor = ColorClasses::green;
  }

  LINE("module:ObjectPerceptor:jerseyScan",
       centerPosImageX,
       player.jerseyY0,
       centerPosImageX,
       player.jerseyY1,
       3,
       Drawings::ps_solid,
       opponentPlotColor);
}

void DeepPlayerPerceptor::displayInferenceResult(DeepPlayerPerceptor::InferenceResult const& inference_result) {

  for (auto const& bbox_with_score : inference_result) {
    auto const color = ColorRGBA(0, 0, 255);

    auto const y1 = bbox_with_score.first[0];
    auto const x1 = bbox_with_score.first[1];
    auto const y2 = bbox_with_score.first[2];
    auto const x2 = bbox_with_score.first[3];

    auto const prob = bbox_with_score.second;

    RECTANGLE("module:DeepPlayerPerceptor:bboxes", x1, y1, x2, y2, 1, Drawings::ps_solid, color);
    DRAWTEXT("module:DeepPlayerPerceptor:scores", (x1 + x2) / 2, (y1 + y2) / 2, 10, ColorClasses::yellow, (int)(prob * 100));
  }
}

void DeepPlayerPerceptor::update(PlayerPercept& playerPercept) {
  DECLARE_DEBUG_DRAWING("module:DeepPlayerPerceptor:bboxes", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:DeepPlayerPerceptor:scores", "drawingOnImage");

  playerPercept.players.clear();

  // Don't run inference on the lower image
  if (theCameraInfo.camera == CameraInfo::lower) {
    return;
  }

  // Get the pointer to the input tensor as float
  float* input_tensor = interpreter_->typed_input_tensor<float>(0);

  // clang-format off
  STOP_TIME_ON_REQUEST("DeepPlayerPerceptor::preprocessing", {
    preprocessAndLoadInput(input_tensor);
  });
  // clang-format on

  // Run inference
  // clang-format off
  STOP_TIME_ON_REQUEST("DeepPlayerPerceptor::inference", {
    VERIFY(interpreter_->Invoke() == kTfLiteOk);
  });
  // clang-format on

  auto inference_result = extractInferenceResultFromOutputTensor();

  if (inference_result.empty()) {
    return;
  }

  // Update PlayerPercept with inference result
  for (auto const& bbox_and_score : inference_result) {

    auto const bbox = bbox_and_score.first;

    PlayerPercept::Player player;
    player.x1 = bbox[1];
    player.x2 = bbox[3];
    player.y1 = bbox[0];
    player.y2 = bbox[2];

    Vector2<> leftInImage = theImageCoordinateSystem.toCorrected(Vector2<>(player.x1, player.y2));
    Vector2<> rightInImage = theImageCoordinateSystem.toCorrected(Vector2<>(player.x2, player.y2));
    Vector2<> leftOnField;
    Vector2<> rightOnField;
    Geometry::calculatePointOnField(Vector2<int>(leftInImage.x, leftInImage.y), theCameraMatrix, theCameraInfo, leftOnField);
    Geometry::calculatePointOnField(
      Vector2<int>(rightInImage.x, rightInImage.y), theCameraMatrix, theCameraInfo, rightOnField);
    player.centerOnField = (leftOnField + rightOnField) * 0.5f;
    player.radius = (leftOnField - rightOnField).abs();
    player.standing = (player.x2 - player.x1 < playerFallenMinimumWidth);

    const float robotHeight = 560; // mm, when standing
    Vector3<> topInWorld(player.centerOnField.x, player.centerOnField.y, robotHeight);
    Vector2<int> topInImage;
    Geometry::calculatePointInImage(topInWorld, theCameraMatrix, theCameraInfo, topInImage);

    if (player.standing) {
      int jerseyY0 = (int)std::round(player.y2 - playerJerseyY[0] * (player.y2 - topInImage.y));
      int jerseyY1 = (int)std::round(player.y2 - playerJerseyY[1] * (player.y2 - topInImage.y));
      int horizonYCoord = static_cast<int>(theImageCoordinateSystem.origin.y);

      player.y1 = std::max(0, topInImage.y);

      player.jerseyY0 = jerseyY0 - std::abs(jerseyY1 - horizonYCoord) / 2;
      player.jerseyY1 = horizonYCoord;
    }

    // Find out whether opponent or not depending on jersey color
    player.detectedJersey = false;
    player.opponent = true;

    if (applyJerseyScan) {
      runJerseyScan(player);
    }

    playerPercept.players.push_back(player);
  }

  // Dislay the inference result
  displayInferenceResult(inference_result);
}
