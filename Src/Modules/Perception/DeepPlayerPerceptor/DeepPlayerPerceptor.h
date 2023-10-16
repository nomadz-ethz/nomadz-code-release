/**
 * @file DeepPlayerPerceptor.h
 *
 * Loads an SSD model to perform robot detection.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Perception/PlayerPercept.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/RefereePercept.h"

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/model.h"

MODULE(DeepPlayerPerceptor)
REQUIRES(Image)
REQUIRES(FrameInfo)
REQUIRES(CameraInfo)
REQUIRES(ImageCoordinateSystem)
REQUIRES(CameraMatrix)
REQUIRES(FieldBoundary)
USES(RefereePercept)
REQUIRES(PlayerPercept)
REQUIRES(FieldDimensions)
REQUIRES(OwnTeamInfo)
USES(RobotPose)
PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(PlayerPercept)
LOADS_PARAMETER(std::string, pathToModel)
LOADS_PARAMETER(unsigned int, inputWidth)
LOADS_PARAMETER(unsigned int, inputHeight)
LOADS_PARAMETER(unsigned int, inputDepth)
LOADS_PARAMETER(float, minimumDetectionScore)
LOADS_PARAMETER(float, playerFallenMinimumWidth)
LOADS_PARAMETER(bool, applyJerseyScan)
LOADS_PARAMETER(std::vector<float>, playerJerseyY)
LOADS_PARAMETER(float, ownJerseyRatioThresh)
LOADS_PARAMETER(float, redHueLowThresh)
LOADS_PARAMETER(float, redHueHighThresh)
LOADS_PARAMETER(float, blueHueLowThresh)
LOADS_PARAMETER(float, blueHueHighThresh)
LOADS_PARAMETER(float, blueSaturationThresh)
LOADS_PARAMETER(float, blueIntensityThresh)
LOADS_PARAMETER(int, centerDistanceToScan)
LOADS_PARAMETER(float, ownJerseyRatioBlackThresh)
LOADS_PARAMETER(float, ownJerseyRatioOrangeThresh)
LOADS_PARAMETER(float, ownJerseyRatioRedThresh)
LOADS_PARAMETER(float, ownJerseyRatioBlueThresh)
END_MODULE

class DeepPlayerPerceptor : public DeepPlayerPerceptorBase {
public:
  DeepPlayerPerceptor();
  ~DeepPlayerPerceptor() = default;

  void update(PlayerPercept& playerPercept);

protected:
  /**
   *
   */
  void preprocessAndLoadInput(float* input_tensor);

  // TODO: implement a proper data structure for this in the base library
  using NormalizedBoundingBox = std::array<float, 4>;
  using BoundingBox = std::array<int, 4>;
  using InferenceResult = std::vector<std::pair<BoundingBox, float>>;

  /**
   * TODO: add docstring
   */
  static BoundingBox
  computeDenormalizedBoundingBox(NormalizedBoundingBox const& normalized_bbox, int image_width, int image_height);

  /**
   * TODO: add docstring
   */
  InferenceResult extractInferenceResultFromOutputTensor();

  /**
   * TODO: add docstring
   */
  static void displayInferenceResult(InferenceResult const& inference_result);

  /**
   * TODO: add docstring
   */
  void runJerseyScan(PlayerPercept::Player& player);

private:
  static constexpr double kNormalizationAlpha = 2.0 / 255.0;
  static constexpr double kNormalizationBeta = -1.0;

  enum OutputTensorType : int { NUM_DETECTIONS = 0, SCORES = 1, CLASSES = 2, BOXES = 3 };

  std::unique_ptr<tflite::FlatBufferModel> model_;
  std::unique_ptr<tflite::Interpreter> interpreter_;

  std::map<OutputTensorType, unsigned int> output_tensor_idxs;
};
