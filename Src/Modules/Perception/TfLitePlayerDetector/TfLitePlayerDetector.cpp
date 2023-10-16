#include "TfLitePlayerDetector.h"

#include <algorithm>
#include <cmath>

#include "Core/System/File.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"

TfLitePlayerDetector::TfLitePlayerDetector() {
  const auto absolutePathToModel = std::string(File::getBHDir()) + "/" + pathToModel;
  detector_ = std::make_unique<YOLOV8Detector>(absolutePathToModel);
}

void TfLitePlayerDetector::update(PlayerPercept& playerPercept) {
  if (theRefereePercept.runModel) {
    return;
  }
  
  DECLARE_DEBUG_DRAWING("module:TfLitePlayerDetector:boxes", "drawingOnImage");

  playerPercept.players.clear();

  std::vector<std::pair<YOLOV8Detector::BoundingBox, float>> boundingBoxesWithScores;
  STOP_TIME_ON_REQUEST("TfLitePlayerDetector::detect", { boundingBoxesWithScores = detector_->detect(theImage, minScore); });

  // Convert detections to Players
  playerPercept.players.reserve(boundingBoxesWithScores.size());
  for (const auto& bbox_w_score : boundingBoxesWithScores) {
    const auto& bbox = bbox_w_score.first;
    const auto& score = bbox_w_score.second;
    const auto& player = createPlayerFromDetection(bbox, score);
    playerPercept.players.push_back(player);
  }

  // Remove players that are outside the field boundary
  std::remove_if(playerPercept.players.begin(), playerPercept.players.end(), [this](const auto& player) {
    return !Geometry::calculatePixelInsideBoundary(
      player.centerOnField.x, player.centerOnField.y, theFieldBoundary, theCameraInfo);
  });

  if (detectTeam) {
    std::for_each(
      playerPercept.players.begin(), playerPercept.players.end(), [this](auto& player) { detectPlayerTeam(player); });
  }
}

PlayerPercept::Player TfLitePlayerDetector::createPlayerFromDetection(const YOLOV8Detector::BoundingBox& box,
                                                                      const float score) const {
  PlayerPercept::Player player;
  player.x1 = box[0];
  player.y1 = box[1];
  player.x2 = box[2];
  player.y2 = box[3];
  player.probability = score;
  Vector2<> leftInImage = theImageCoordinateSystem.toCorrected(Vector2<>(player.x1, player.y2));
  Vector2<> rightInImage = theImageCoordinateSystem.toCorrected(Vector2<>(player.x2, player.y2));
  Vector2<> leftOnField;
  Vector2<> rightOnField;
  Geometry::calculatePointOnField(Vector2<int>(leftInImage.x, leftInImage.y), theCameraMatrix, theCameraInfo, leftOnField);
  Geometry::calculatePointOnField(
    Vector2<int>(rightInImage.x, rightInImage.y), theCameraMatrix, theCameraInfo, rightOnField);
  player.centerOnField = (leftOnField + rightOnField) * 0.5f;
  player.radius = (leftOnField - rightOnField).abs();
  player.standing = (player.x2 - player.x1 < playerFallenMinWidth);
  player.detectedJersey = false;
  player.opponent = true;

  // Adjust the top of the bounding box to the expected height of a robot
  if (player.standing) {
    Vector3<> topInWorld(player.centerOnField.x, player.centerOnField.y, ROBOT_HEIGHT_MM);
    Vector2<int> topInImage;
    Geometry::calculatePointInImage(topInWorld, theCameraMatrix, theCameraInfo, topInImage);
    player.y1 = std::max(0, topInImage.y);

    int jerseyY0 = player.y2;
    int jerseyY1 = topInImage.y;
    int horizonYCoord = static_cast<int>(theImageCoordinateSystem.origin.y);

    player.jerseyY0 = jerseyY0 - std::abs(jerseyY1 - horizonYCoord) / 2;
    player.jerseyY1 = horizonYCoord;
  }

  return player;
}

void TfLitePlayerDetector::detectPlayerTeam(PlayerPercept::Player& player) {
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

      // RGB -> HSI
      float hue, saturation, intensity;
      ColorModelConversions::fromYCbCrToHSI2(yValue, cbValue, crValue, hue, saturation, intensity);

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
}

MAKE_MODULE(TfLitePlayerDetector, Perception)