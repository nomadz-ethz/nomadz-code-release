/**
 * @file TfLiteBallPerceptor.h
 *
 * Loads a CNN to classify balls using TFLite
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/PlayerPercept.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Geometry/Transformations.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/model.h"

MODULE(TfLiteBallPerceptor)
REQUIRES(Image)
REQUIRES(FrameInfo)
REQUIRES(CameraInfo)
REQUIRES(ImageCoordinateSystem)
REQUIRES(CameraMatrix)
REQUIRES(FieldDimensions)
REQUIRES(BallSpots)
REQUIRES(PlayerPercept)
USES(RobotPose)
PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallPercept)
LOADS_PARAMETER(std::string, pathToModel)
LOADS_PARAMETER(unsigned int, patchWidth)
LOADS_PARAMETER(unsigned int, patchHeight)
LOADS_PARAMETER(double, marginFactor)
LOADS_PARAMETER(double, minBallProb)
LOADS_PARAMETER(bool, useBallOutsideFieldCheck)
LOADS_PARAMETER(float, ballOutsideFieldOffset)
LOADS_PARAMETER(bool, useBallWithinPlayerCheck)
LOADS_PARAMETER(float, ballPlayerAllowedPercentage)
LOADS_PARAMETER(bool, applyNormalization)
LOADS_PARAMETER(bool, useRadiusRegression)
LOADS_PARAMETER(float, minPatchesDistance)
END_MODULE

class TfLiteBallPerceptor : public TfLiteBallPerceptorBase {

public:
  /**
   * Loads the network and configures the parameters for inference
   * i.e. input shape and class labels.
   */
  TfLiteBallPerceptor();

  ~TfLiteBallPerceptor() {}

  /**
   * Update the ball perception from the network.
   */

  void update(BallPercept& ballPercept);

private:
  struct BallDetectionResult {
    std::vector<float> scores;
    std::vector<Geometry::Circle> circles;
  };

  /**
   * Pass patch through the network and compute ball scores.
   */
  double predict(int x1, int x2, int y1, int y2);

  /**
   * Remove ball spots which are outside the field or inside a robot.
   */
  std::vector<BallSpot> filterBallSpots(const std::vector<BallSpot>& ballSpots);

  /**
   * Loop over the ball region proposals and classify them with the neural network.
   */
  std::map<const BallSpot*, double> labelBallSpots(const std::vector<BallSpot>& ballSpots);
  /**
   * Register patches drawing patches on the image for representation in SimRobot
   */
  void drawPatches(std::map<const BallSpot*, double> spots);

  bool ballOutsideOfField(Vector2<> relPosOnField);

  std::unique_ptr<tflite::FlatBufferModel> model_;
  std::unique_ptr<tflite::Interpreter> interpreter_;
};
