#pragma once

#include "Core/Module/Module.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/PlayerPercept.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/RefereePercept.h"

#include "YOLOV8Detector.h"

MODULE(TfLitePlayerDetector)
REQUIRES(CameraInfo)
REQUIRES(CameraMatrix)
REQUIRES(FieldBoundary)
REQUIRES(Image)
USES(RefereePercept)
REQUIRES(ImageCoordinateSystem)
REQUIRES(OwnTeamInfo)
PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(PlayerPercept)
LOADS_PARAMETER(std::string, pathToModel)
LOADS_PARAMETER(float, minScore)
LOADS_PARAMETER(float, playerFallenMinWidth)
LOADS_PARAMETER(bool, detectTeam)
LOADS_PARAMETER(int, centerDistanceToScan)
LOADS_PARAMETER(float, ownJerseyRatioThresh)
LOADS_PARAMETER(float, redHueLowThresh)
LOADS_PARAMETER(float, redHueHighThresh)
LOADS_PARAMETER(float, blueHueLowThresh)
LOADS_PARAMETER(float, blueHueHighThresh)
LOADS_PARAMETER(float, blueSaturationThresh)
LOADS_PARAMETER(float, blueIntensityThresh)
LOADS_PARAMETER(float, ownJerseyRatioBlackThresh)
LOADS_PARAMETER(float, ownJerseyRatioOrangeThresh)
LOADS_PARAMETER(float, ownJerseyRatioRedThresh)
LOADS_PARAMETER(float, ownJerseyRatioBlueThresh)
END_MODULE

class TfLitePlayerDetector : public TfLitePlayerDetectorBase {
public:
  TfLitePlayerDetector();

  void update(PlayerPercept& playerPercept);

private:
  static constexpr float ROBOT_HEIGHT_MM = 560.0f;

  PlayerPercept::Player createPlayerFromDetection(const YOLOV8Detector::BoundingBox& box, const float score) const;

  void detectPlayerTeam(PlayerPercept::Player& player);

  std::unique_ptr<YOLOV8Detector> detector_;
};