/**
 * @file TfLiteBallPerceptor.cpp
 *
 * Implements module TfLiteBallPerceptor.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include "TfLiteBallPerceptor.h"

#include "Core/System/File.h"
#include "Representations/Infrastructure/Image.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"

#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/optional_debug_tools.h"

// Prints additional details about the network.
// #define TFLITE_DEBUG

MAKE_MODULE(TfLiteBallPerceptor, Perception)

TfLiteBallPerceptor::TfLiteBallPerceptor() {
  // Load the model and initialize interpreter
  auto absolutePath = std::string(File::getBHDir()) + "/" + pathToModel;
  model_ = tflite::FlatBufferModel::BuildFromFile(absolutePath.c_str());
  ASSERT(model_ != nullptr);

  tflite::ops::builtin::BuiltinOpResolver resolver;
  tflite::InterpreterBuilder builder(*model_, resolver);
  builder(&interpreter_);
  VERIFY(interpreter_->AllocateTensors() == kTfLiteOk);

#ifdef TFLITE_DEBUG
  OUTPUT_TEXT("TfLiteBallPerceptor: Loaded model and initialized interpreter!");
  EXECUTE_ONLY_IN_DEBUG(tflite::PrintInterpreterState(interpreter_.get()););
#endif

  // Fetch input shape and check dimensions
  if (useRadiusRegression) {
    VERIFY(interpreter_->inputs().size() == 1 && interpreter_->outputs().size() == 2);
  } else {

    VERIFY(interpreter_->inputs().size() == 1 && interpreter_->outputs().size() == 1);
  }

  // Validate input shape
  auto input = interpreter_->input_tensor(0);
  VERIFY(input->dims->size == 4 && input->dims->data[0] == 1 && input->dims->data[1] == patchWidth &&
         input->dims->data[2] == patchHeight && input->dims->data[3] == 1);

  // Validate output shape
  if (useRadiusRegression) {
    auto output_1 = interpreter_->output_tensor(0);
    VERIFY(output_1->dims->size == 2 && output_1->dims->data[0] == 1 && output_1->dims->data[1] == 1);

    auto output_2 = interpreter_->output_tensor(1);
    VERIFY(output_1->dims->size == 2 && output_2->dims->data[0] == 1 && output_2->dims->data[1] == 3);

  } else {
    auto output = interpreter_->output_tensor(0);
    VERIFY(output->dims->size == 2 && output->dims->data[0] == 1 && output->dims->data[1] == 1);
  }
}

double TfLiteBallPerceptor::predict(int x1, int x2, int y1, int y2) {
  // Fill input buffers
  float* input = interpreter_->typed_input_tensor<float>(0);
  for (int x = x1; x <= x2; ++x) {
    for (int y = y1; y <= y2; ++y) {
      int cx = static_cast<double>(x - x1) / (x2 - x1) * (patchWidth - 1);
      int cy = static_cast<double>(y - y1) / (y2 - y1) * (patchHeight - 1);

      if (0 <= x && x < theImage.width && 0 <= y && y < theImage.height) {
        if (applyNormalization) {
          input[cy * patchWidth + cx] = theImage[y][x].y / 255.0;
        } else {
          input[cy * patchWidth + cx] = theImage[y][x].y;
        }
      } else {
        input[cy * patchWidth + cx] = 0.0;
      }
    }
  }

  // Run inference
  VERIFY(interpreter_->Invoke() == kTfLiteOk);

  // Retrieve the score
  float score = interpreter_->typed_output_tensor<float>(0)[0];

  // TODO: parse, return and use this
  return score;
}

bool TfLiteBallPerceptor::ballOutsideOfField(Vector2<> relativeFieldPos) {
  Vector2<> globalBallPos = theRobotPose * relativeFieldPos;

  // xAxis check
  if ((globalBallPos.x > (theFieldDimensions.xPosOpponentGroundline + ballOutsideFieldOffset)) ||
      (globalBallPos.x < (theFieldDimensions.xPosOwnGroundline - ballOutsideFieldOffset))) {
    return true;
  }

  // yAxis check
  if ((globalBallPos.y > (theFieldDimensions.yPosLeftSideline + ballOutsideFieldOffset)) ||
      (globalBallPos.y < (theFieldDimensions.yPosRightSideline - ballOutsideFieldOffset))) {
    return true;
  }

  return false;
}

std::map<const BallSpot*, double> TfLiteBallPerceptor::labelBallSpots(const std::vector<BallSpot>& ballSpots) {
  // Clear the probs and labels buffers for a new round
  std::map<const BallSpot*, double> ballSpotProbs;

  // Iterate over the region proposals - measure time
  STOP_TIME_ON_REQUEST("TfLiteBallPerceptor::labelBallSpots", {
    for (const auto& spot : ballSpots) {
      // Get the radius and position (in 2d) of the circular region
      int cr = (int)std::round(spot.radius * (1.0f + marginFactor));
      int cx = std::max(cr, std::min(spot.position.x, theImage.width - 1 - cr));
      int cy = std::max(cr, std::min(spot.position.y, theImage.height - 1 - cr));

      // // Define the corners of the rectangular region to extract from the image
      int x1 = cx - cr;
      int x2 = cx + cr;
      int y1 = cy - cr;
      int y2 = cy + cr;

      // Run inference
      double score = predict(x1, x2, y1, y2);

      // Set probabilities
      ballSpotProbs.insert(std::make_pair(&spot, score));
    }
  });

  return ballSpotProbs;
}

std::vector<BallSpot> TfLiteBallPerceptor::filterBallSpots(const std::vector<BallSpot>& ballSpots) {
  std::vector<BallSpot> filteredBallSpots;
  bool upperImage = theCameraInfo.camera == CameraInfo::upper;
  for (int i = 0; i < ballSpots.size(); ++i) {
    auto spot = ballSpots[i];

    Vector2<> ballBottomOnImage =
      theImageCoordinateSystem.toCorrected(Vector2<>(spot.position.x, spot.position.y + spot.radius));
    Vector2<> relFieldPos;
    Geometry::calculatePointOnField(ballBottomOnImage, theCameraMatrix, theCameraInfo, relFieldPos);
    if (upperImage && useBallOutsideFieldCheck && (theRobotPose.validity > 0.3) && ballOutsideOfField(relFieldPos)) {
      continue;
    }
    if (upperImage && useFieldBoundaryRejection &&
        !Geometry::calculatePixelInsideBoundary(ballBottomOnImage.x, ballBottomOnImage.y, theFieldBoundary, theCameraInfo)) {
      continue;
    }
    ///*** Using the new method for checking if the image is inside the image or not
    if (useBallWithinPlayerCheck) {
      bool withinPlayer = false;
      for (const auto& player : thePlayerPercept.players) {
        float offX = ballPlayerAllowedPercentage * (player.x2 - player.x1) / 2;
        float offY = ballPlayerAllowedPercentage * (player.y2 - player.y1) / 2;
        if (player.x1 + offX < spot.position.x && spot.position.x < player.x2 - offX && player.y1 + offY < spot.position.y &&
            spot.position.y < player.y2 - offY) {
          withinPlayer = true;
        }
      }
      if (withinPlayer) {
        continue;
      }
    }

    bool skipTooClose = false;
    for (int j = 0; j < i; ++j) {
      auto checkSpot = ballSpots[j];
      auto centerDist = std::hypot(spot.position.x - checkSpot.position.x, spot.position.y - checkSpot.position.y);
      if (centerDist < minPatchesDistance) {
        skipTooClose = true;
        break;
      }
    }

    if (skipTooClose) {
      continue;
    }

    filteredBallSpots.push_back(spot);
  }

  return filteredBallSpots;
}

void TfLiteBallPerceptor::update(BallPercept& ballPercept) {
  DECLARE_DEBUG_DRAWING("module:TfLiteBallPerceptor:patches", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:TfLiteBallPerceptor:probs", "drawingOnImage");

  std::vector<BallSpot> ballSpots = theBallSpots.ballSpots;
  std::vector<BallSpot> filteredBallSpots = filterBallSpots(ballSpots);

  auto spotProbs = labelBallSpots(filteredBallSpots);
  drawPatches(spotProbs);

  ballPercept.ballWasSeen = false;
  double maxBallProb = 0.0;
  for (const auto& spotProb : spotProbs) {
    auto spot = *spotProb.first;
    auto prob = spotProb.second;
    if (prob < minBallProb || prob < maxBallProb) {
      continue;
    }

    BallPercept ballProp;
    ballProp.ballWasSeen = true;
    ballProp.positionInImage = Vector2<>(spot.position.x, spot.position.y);
    ballProp.radiusInImage = static_cast<float>(spot.radius);
    ballPercept.timeWhenLastSeen = static_cast<unsigned int>(theFrameInfo.time);

    Vector2<> ballBottomOnImage =
      theImageCoordinateSystem.toCorrected(Vector2<>(spot.position.x, spot.position.y + ballPercept.radiusInImage));
    Geometry::calculatePointOnField(ballBottomOnImage, theCameraMatrix, theCameraInfo, ballProp.relativePositionOnField);

    ballPercept = ballProp;
    maxBallProb = prob;
  }
}

void TfLiteBallPerceptor::drawPatches(std::map<const BallSpot*, double> spotProbs) {
  for (const auto& spotProb : spotProbs) {
    auto spot = *spotProb.first;
    auto prob = spotProb.second;
    const ColorRGBA color = ColorRGBA(255, 0, 0);

    const int cr = (int)std::round(spot.radius * (1.0f + marginFactor));
    const int cx = spot.position.x;
    const int cy = spot.position.y;
    const int r = static_cast<int>(spot.radius);

    const int x1 = cx - cr;
    const int x2 = cx + cr;
    const int y1 = cy - cr;
    const int y2 = cy + cr;

    CIRCLE("module:TfLiteBallPerceptor:patches", cx, cy, r, 1, Drawings::ps_solid, color, Drawings::bs_null, ColorRGBA());
    RECTANGLE("module:TfLiteBallPerceptor:patches", x1, y1, x2, y2, 1, Drawings::ps_solid, color);

    DRAWTEXT("module:TfLiteBallPerceptor:probs", (x1 + x2) / 2, (y1 + y2) / 2, 10, ColorClasses::blue, (int)(prob * 100));
  }
}
