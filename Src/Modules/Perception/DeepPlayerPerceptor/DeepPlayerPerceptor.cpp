/**
 * @file DeepPlayerPerceptor.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include "DeepPlayerPerceptor.h"
#include "Core/System/File.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"

#include "Representations/Infrastructure/Image.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/optional_debug_tools.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <cmath>

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
  interpreter_->SetNumThreads(2);

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

  auto const input_height = interpreter_->input_tensor(0)->dims->data[1];
  auto const input_width = interpreter_->input_tensor(0)->dims->data[2];

  Image rescaled_input = theImage.resizeAndFillUpInterLinear(input_width, input_height);

  // Load in the input tensor
  for (size_t x = 0; x < input_width; ++x) {
    for (size_t y = 0; y < input_height; ++y) {
      input_tensor[y * input_width + x] = rescaled_input[y][x].y / 127.5f - 1;
    }
  };
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

  int centerPosImageX = int((player.x2 + player.x1) / 2.0f);

  float ownJerseyCounterRed = 0;
  float ownJerseyCounterBlue = 0;
  float ownJerseyCounterBlack = 0;
  float ownJerseyCounterOrange = 0;
  float numOfPx = 0;

  // compute jersey scan region boundaries
  const auto leftBorder = std::max(0, std::max(static_cast<int>(player.x1), centerPosImageX - centerDistanceToScan));
  const auto rightBorder =
    std::min(theImage.width - 1, std::min(centerPosImageX + centerDistanceToScan, static_cast<int>(player.x2)));
  const auto topBorder = std::max(0, static_cast<int>(player.jerseyY1));
  const auto bottomBorder = std::min(theImage.height - 1, static_cast<int>(player.jerseyY0));

  for (int scanIterY = topBorder; scanIterY < bottomBorder; ++scanIterY) {
    for (int scanIterX = leftBorder; scanIterX < rightBorder; ++scanIterX) {
      const Image::Pixel px = theImage[scanIterY][scanIterX];

      int yValue = static_cast<int>(px.y);
      int cbValue = static_cast<int>(px.cb);
      int crValue = static_cast<int>(px.cr);

      unsigned char r, g, b;
      ColorModelConversions::fromYCbCrToRGB(yValue, cbValue, crValue, r, g, b);

      // RGB -> HSI
      float hue;
      float minRGB = std::min({r, g, b});
      if (r == g && g == b) { /// If r=g=b, the color is a shade of gray and the hue is undefined
        hue = 0.0f;
      } else {
        if (b > g) {
          hue = 360 - std::acos((r - 0.5 * g - 0.5 * b) /
                                std::sqrt(std::pow(r, 2) + std::pow(g, 2) + std::pow(b, 2) - r * g - r * b - g * b)) /
                        pi_180;
        } else {
          hue = std::acos((r - 0.5 * g - 0.5 * b) /
                          std::sqrt(std::pow(r, 2) + std::pow(g, 2) + std::pow(b, 2) - r * g - r * b - g * b)) /
                pi_180;
        }
      }
      float intensity = (r + g + b) / 3;

      float saturation;
      if (intensity > 0) {
        saturation = 1 - (minRGB / intensity);
      } else {
        saturation = 0;
      }

      numOfPx++;

      if ((hue < redHueHighThresh || hue > redHueLowThresh) && (saturation > 0.4)) {
        ownJerseyCounterRed++;
      }
      if (intensity < 25) {
        ownJerseyCounterBlack++;
      }
      if ((hue < redHueHighThresh || hue > redHueLowThresh) && (saturation > 0.5) && (intensity < 140)) {
        ownJerseyCounterOrange++;
      }
      if ((hue > blueHueLowThresh && hue < blueHueHighThresh) && (saturation < blueSaturationThresh) &&
          (intensity < blueIntensityThresh)) {
        ownJerseyCounterBlue++;
      }
    }
  }

  float ownJerseyRatioBlack = 0.f, ownJerseyRatioRed = 0.f, ownJerseyRatioBlue = 0.f, ownJerseyRatioOrange = 0.f;
  if (numOfPx > 0) {
    ownJerseyRatioBlack = ownJerseyCounterBlack / numOfPx;
    ownJerseyRatioRed = ownJerseyCounterRed / numOfPx;
    ownJerseyRatioBlue = ownJerseyCounterBlue / numOfPx;
    ownJerseyRatioOrange = ownJerseyCounterOrange / numOfPx;
  }

  // if ownJerseyRatio is smaller than 0.02 jersey is home team 0, else opponent team 1
  if (theOwnTeamInfo.teamColor == TEAM_BLUE) { // Assumes that this is the home team with jersey color blue and goalkeeper
                                               // orange
    if (ownJerseyRatioBlue > ownJerseyRatioBlueThresh) {
      player.detectedJersey = true;
      player.opponent = false;
      player.isGoalKeeper = false;
      player.jerseyRatio = ownJerseyRatioBlue;
    }
  }
  if (theOwnTeamInfo.goalkeeperColor == TEAM_ORANGE) {
    if (ownJerseyRatioOrange > ownJerseyRatioOrangeThresh) {
      player.detectedJersey = true;
      player.opponent = false;
      player.isGoalKeeper = true;
      player.jerseyRatio = ownJerseyRatioOrange;
    }
  }
  if (theOwnTeamInfo.teamColor == TEAM_RED) { // Assumes that this is the home team with jersey color red and goalkeeper
                                              // black
    if (ownJerseyRatioRed > ownJerseyRatioRedThresh) {
      player.detectedJersey = true;
      player.opponent = false;
      player.isGoalKeeper = false;
      player.jerseyRatio = ownJerseyRatioRed;
    }
  }
  if (theOwnTeamInfo.goalkeeperColor == TEAM_BLACK) {
    if (ownJerseyRatioBlack > ownJerseyRatioBlackThresh) {
      player.detectedJersey = true;
      player.opponent = false;
      player.isGoalKeeper = true;
      player.jerseyRatio = ownJerseyRatioBlack;
    }
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

void DeepPlayerPerceptor::update(PlayerPercept& playerPercept) {
  if (theRefereePercept.runModel) {
    return;
  }

  DECLARE_DEBUG_DRAWING("module:DeepPlayerPerceptor:bboxes", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:DeepPlayerPerceptor:scores", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:DeepPlayerPerceptor:dots", "drawingOnImage");

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
    auto const prob = bbox_and_score.second;
    PlayerPercept::Player player;
    player.x1 = bbox[1];
    player.x2 = bbox[3];
    player.y1 = bbox[0];
    player.y2 = bbox[2];
    player.probability = prob;
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
    if (Geometry::calculatePixelInsideBoundary(
          player.centerOnField.x, player.centerOnField.y, theFieldBoundary, theCameraInfo)) {
      continue;
    }
    const float robotHeight = 560; // mm, when standing
    Vector3<> topInWorld(player.centerOnField.x, player.centerOnField.y, robotHeight);
    Vector2<int> topInImage;
    Geometry::calculatePointInImage(topInWorld, theCameraMatrix, theCameraInfo, topInImage);

    if (player.standing) {
      int jerseyY0 = static_cast<int>(std::round(player.y2 - playerJerseyY[0] * (player.y2 - topInImage.y)));
      int jerseyY1 = static_cast<int>(std::round(player.y2 - playerJerseyY[1] * (player.y2 - topInImage.y)));
      int horizonYCoord = static_cast<int>(theImageCoordinateSystem.origin.y);

      player.y1 = std::max(0, topInImage.y);

      player.jerseyY0 = jerseyY0 - std::abs(jerseyY1 - horizonYCoord) / 2;
      player.jerseyY1 = horizonYCoord;
    }

    // Find out whether opponent or not depending on jersey color
    player.detectedJersey = false;
    player.opponent = true;
    auto color = ColorRGBA(255, 0, 0);
    if (applyJerseyScan) {
      runJerseyScan(player);
    }

    RECTANGLE("module:DeepPlayerPerceptor:bboxes",
              player.x1,
              player.jerseyY0,
              player.x2,
              player.jerseyY1,
              1,
              Drawings::ps_solid,
              color);
    DRAWTEXT("module:DeepPlayerPerceptor:scores",
             (player.x1 + player.x2) / 2,
             (player.y1 + player.y2) / 2,
             30,
             ColorClasses::yellow,
             (int)(player.opponent));
    DRAWTEXT("module:DeepPlayerPerceptor:scores",
             (player.x1 + player.x2) / 2,
             (player.y2 + player.y2) / 2,
             15,
             ColorClasses::yellow,
             (int)(player.jerseyRatio * 1000000));
    playerPercept.players.push_back(player);
  }
}
