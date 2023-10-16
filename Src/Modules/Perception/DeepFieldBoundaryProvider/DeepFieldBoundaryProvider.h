/**
 * @file DeepFieldBoundaryProvider.h
 *
 * Loads a NN model to perform field boundary detection and least squares fitting to find the field boundary.
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
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Perception/PlayerPercept.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Tools/Geometry/Transformations.h"
#include "Representations/Perception/RefereePercept.h"
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/model.h"

MODULE(DeepFieldBoundaryProvider)
REQUIRES(Image)
REQUIRES(FieldBoundary)
REQUIRES(CameraInfo)
REQUIRES(FieldDimensions)
REQUIRES(OwnTeamInfo)
USES(RefereePercept)
PROVIDES_WITH_OUTPUT_AND_DRAW(FieldBoundary)
LOADS_PARAMETER(std::string, pathToModel)
END_MODULE

class DeepFieldBoundaryProvider : public DeepFieldBoundaryProviderBase {
public:
  DeepFieldBoundaryProvider();
  ~DeepFieldBoundaryProvider() = default;

  void update(FieldBoundary& fieldBoundary);

protected:
  /**
   *
   */
  void preprocessAndLoadInput(
    float* input_tensor); ///< Converts the YCbCr channels of the image to a cv mat and loads it into the input tensor
  void displayInferenceResults(const std::vector<float>& model); ///< Draws the field boundary on the image
  std::vector<float> fitLine(const std::vector<float>& x, const std::vector<float>& y); ///< Fits a line to the given points
  std::vector<float> findBestModel(const std::vector<float>& x,
                                   const std::vector<float>& y); ///< Finds the best model for the given points
  float getFieldBoundaryY(const std::vector<float>& model,
                          float x); ///< Calculates the y value of the field boundary for the given x value
  std::vector<float> getColumnWiseY(const std::vector<float>& model,
                                    int x); ///< Calculates the y values of the field boundary for whole image (via columns)

  std::unique_ptr<tflite::FlatBufferModel> model_;
  std::unique_ptr<tflite::Interpreter> interpreter_;
};
