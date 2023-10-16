/**
 * @file ObjectPerceptor.cpp
 *
 * This file implements the module ObjectPerceptor.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include "ObjectPerceptor.h"

#include "Core/System/File.h"
#include "Representations/Infrastructure/Image.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"
#include "Tools/RandomForest/LightPatch.h"

#include <iostream>
#include <sys/stat.h>

MAKE_MODULE(ObjectPerceptor, Perception)

cv::Mat ObjectPerceptor::cropImage(const Image& image, const cv::Rect& region) {
  cv::Mat input(image.height,
                image.width,
                CV_8UC4,
                (void*)&(image[0][0].channels[0]),
                image.widthStep * sizeof(decltype(image.widthStep)));
  cv::Mat cropped = input(region);

  cv::Mat mat(cropped.rows, cropped.cols, CV_8UC3);
  cv::mixChannels({cropped},
                  {mat},
                  {
                    2, // Pixel.y
                    0, //  -> out[0]
                    3, // Pixel.cr
                    1, //  -> out[1]
                    1, // Pixel.cb
                    2  //  -> out[2]
                  });

  return mat;
}

void ObjectPerceptor::labelBallSpotsOnce() {
  // Only label once per frame
  if (lastLabeled == theFrameInfo.time) {
    return;
  }

  const LightForest* forest = theRandomForests.get("allCircles");
  if (!forest) {
    return;
  }

  const std::map<RandomForestLabel, ColorRGBA> classColors = {
    {RandomForests::negative, ColorRGBA(255, 255, 255)},
    {RandomForests::ball, ColorRGBA(0, 255, 255)},
    {RandomForests::robotBody, ColorRGBA(255, 0, 0)},
    {RandomForests::line, ColorRGBA(255, 165, 0)},
  };

  const int patchSize = 24;
  const float marginFactor = 0.0f; // Extra margins that are 25% the radius (approx sqrt(2*pi)/2 - 1)

  STOP_TIME_ON_REQUEST("ObjectPerceptor::labelBallSpotsOnce", {
    ballSpotProbs.clear();
    ballSpotLabels.clear();

    for (const auto& spot : theBallSpots.ballSpots) {
      // Calculate center coordinates, but make sure entire circle fits within image
      const int cr = (int)std::round(spot.radius * (1.0f + marginFactor));
      const int cx = spot.position.x;
      const int cy = spot.position.y;

      const int x1 = cx - cr;
      const int x2 = cx + cr;
      const int y1 = cy - cr;
      const int y2 = cy + cr;

      LightPatch patch = LightPatch::createRectFromSides(theImage, x1, x2, y1, y2, patchSize);

      auto probs = forest->predict(patch);
      auto label = forest->mostLikelyLabel(probs);

      ballSpotProbs.insert(std::make_pair(&spot, probs));
      ballSpotLabels.insert(std::make_pair(&spot, label));
    }

    lastLabeled = theFrameInfo.time;
  });

  if (excludeRobotsOnBall) {
    // For patches labeled "robot", set label to "negative" if it overlaps with any "ball" patch,
    // and transfer the "robot" class probability into the "negative" class.
    STOP_TIME_ON_REQUEST("ObjectPerceptor::excludeRobotsOnBall", {
      // Find all "ball" patches
      std::vector<const BallSpot*> balls;
      for (const auto& ballSpotLabel : ballSpotLabels) {
        const BallSpot* spot = ballSpotLabel.first;
        const auto& label = ballSpotLabel.second;
        if (label == RandomForests::ball) {
          balls.push_back(spot);
        }
      }

      // "overlap" = circles overlap
      const auto spotsOverlap = [=](const BallSpot& a, const BallSpot& b) {
        const int ra = (int)std::round(a.radius * (1.0f + marginFactor));
        const int rb = (int)std::round(b.radius * (1.0f + marginFactor));

        const int minDst = ra + rb;
        return (a.position - b.position).squareAbs() <= minDst * minDst;
      };

      // Convert matching "robot" patches into "negative"
      for (auto& ballSpotLabel : ballSpotLabels) {
        const BallSpot* spot = ballSpotLabel.first;
        auto& label = ballSpotLabel.second;
        auto& probs = ballSpotProbs.at(spot);

        if (label == RandomForests::robotBody) {
          for (const auto& ball : balls) {
            if (spotsOverlap(*spot, *ball)) {
              label = RandomForests::negative;
              probs[RandomForests::negative] += probs[RandomForests::robotBody];
              probs[RandomForests::robotBody] = 0;
              break;
            }
          }
        }
      }
    });
  }

  // Draw patches
  for (const auto& ballSpotLabel : ballSpotLabels) {
    const BallSpot& spot = *(ballSpotLabel.first);
    const auto& label = ballSpotLabel.second;
    const auto& probs = ballSpotProbs.at(&spot);

    const ColorRGBA color = classColors.at(label);

    const int cr = (int)std::round(spot.radius * (1.0f + marginFactor));
    const int cx = spot.position.x;
    const int cy = spot.position.y;

    const int x1 = cx - cr;
    const int x2 = cx + cr;
    const int y1 = cy - cr;
    const int y2 = cy + cr;

    if (label == RandomForests::ball) {
      RECTANGLE("module:ObjectPerceptor:patches", x1, y1, x2, y2, 1, Drawings::ps_solid, color);
      DRAWTEXT("module:ObjectPerceptor:probs",
               (x1 + x2) / 2,
               (y1 + y2) / 2,
               10,
               ColorClasses::yellow,
               (int)(probs.at(RandomForests::negative) * 100) << "/[" << (int)(probs.at(RandomForests::ball) * 100) << "]/"
                                                              << (int)(probs.at(RandomForests::robotBody) * 100));

    } else if (label == RandomForests::robotBody) {
      RECTANGLE("module:ObjectPerceptor:patches", x1, y1, x2, y2, 1, Drawings::ps_solid, color);
      DRAWTEXT("module:ObjectPerceptor:probs",
               (x1 + x2) / 2,
               (y1 + y2) / 2,
               10,
               ColorClasses::yellow,
               (int)(probs.at(RandomForests::negative) * 100) << "/" << (int)(probs.at(RandomForests::ball) * 100) << "/["
                                                              << (int)(probs.at(RandomForests::robotBody) * 100) << "]");

    } else if (label == RandomForests::line) {
      RECTANGLE("module:ObjectPerceptor:patches", x1, y1, x2, y2, 1, Drawings::ps_solid, color);
      DRAWTEXT("module:ObjectPerceptor:probs",
               (x1 + x2) / 2,
               (y1 + y2) / 2,
               10,
               ColorClasses::yellow,
               (int)(probs.at(RandomForests::negative) * 100) << "/" << (int)(probs.at(RandomForests::ball) * 100) << "/"
                                                              << (int)(probs.at(RandomForests::robotBody) * 100));
    } else {
      RECTANGLE("module:ObjectPerceptor:patches", x1, y1, x2, y2, 1, Drawings::ps_solid, color);
      DRAWTEXT("module:ObjectPerceptor:probs",
               (x1 + x2) / 2,
               (y1 + y2) / 2,
               10,
               ColorClasses::yellow,
               "[" << (int)(probs.at(RandomForests::negative) * 100) << "]/" << (int)(probs.at(RandomForests::ball) * 100)
                   << "/" << (int)(probs.at(RandomForests::robotBody) * 100));
    }
  }
}

void ObjectPerceptor::update(LinePercept& linePercept) {
  DECLARE_DEBUG_DRAWING("module:ObjectPerceptor:patches", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ObjectPerceptor:probs", "drawingOnImage");

  const std::map<RandomForestLabel, ColorRGBA> classColors = {
    {RandomForests::negative, ColorRGBA(255, 255, 255, 85)},
    {RandomForests::line, ColorRGBA(255, 165, 0)},
  };

  linePercept.fieldLineSegments.clear();

  const LightForest* forest = theRandomForests.get("allLines");
  if (!forest) {
    return;
  }

  const int patchSize = 24;
  bool upperImage = theCameraInfo.camera == CameraInfo::upper;
  for (const auto& segment : theShapePercept.segments) {
    const Vector2<> p1(segment.p1.x, segment.p1.y);
    const Vector2<> p2(segment.p2.x, segment.p2.y);

    const Vector2<> mid = (p1 + p2) * 0.5f;
    if (upperImage && useFieldBoundaryRejection) {
      if (!Geometry::calculatePixelInsideBoundary(mid.x, mid.y, theFieldBoundary, theCameraInfo)) {
        continue;
      };

      if (useLineWithinPlayerCheck) {
        bool withinPlayer = false;
        for (const auto& player : thePlayerPercept.players) {
          float offX = linePlayerAllowedPercentage * (player.x2 - player.x1) / 2;
          float offY = linePlayerAllowedPercentage * (player.y2 - player.y1) / 2;
          if (player.x1 + offX < mid.x && mid.x < player.x2 - offX && player.y1 + offY < mid.y && mid.y < player.y2 - offY) {
            withinPlayer = true;
          }
        }
        if (withinPlayer) {
          continue;
        }
      }
    }
    Vector2<> midOnField;
    if (!Geometry::calculatePointOnField(mid, theCameraMatrix, theCameraInfo, midOnField)) {
      continue;
    }

    Vector2<int> slightlyCloserInImage;
    const float realThickness = 250.f; // mm
    midOnField.normalize(midOnField.abs() - realThickness);
    if (!Geometry::calculatePointInImage(midOnField, theCameraMatrix, theCameraInfo, slightlyCloserInImage)) {
      continue;
    }

    const Vector2<> slightlyCloser(slightlyCloserInImage.x,
                                   slightlyCloserInImage.y); // exactly slightlyCloserInImage, but with floats
    const float dir2Len = std::max(patchSize / 3.f, (mid - slightlyCloser).abs());
    Vector2<> dir1 = p2 - p1;
    Vector2<> dir2 = dir1.left() / dir1.abs() *
                     dir2Len; // Actually rotates right, because screen x-y rotation is not trigonometric rotation
    Vector2<> origin = p1 - dir2 * 0.5f;

    const float extraMargin = 4.f; // px: extra pixels in image to take in on each side
    origin -= dir1 / dir1.abs() * extraMargin;
    origin -= dir2 / dir2.abs() * extraMargin;
    dir1.normalize(extraMargin + dir1.abs() + extraMargin);
    dir2.normalize(extraMargin + dir2.abs() + extraMargin);

    LightPatch patch = LightPatch(theImage, origin, dir1 / patchSize, dir2 / patchSize);

    auto probs = forest->predict(patch);
    auto label = forest->mostLikelyLabel(probs);

    const ColorRGBA color = classColors.at(label);

    const std::vector<Vector2<>> patchShape = {origin, origin + dir1, origin + dir1 + dir2, origin + dir2};
    POLYGON("module:ObjectPerceptor:patches",
            (int)patchShape.size(),
            patchShape,
            1,
            Drawings::ps_solid,
            color,
            Drawings::bs_null,
            ColorRGBA());

    if (label == RandomForests::line) {
      linePercept.fieldLineSegments.push_back(segment);

      DRAWTEXT("module:ObjectPerceptor:probs",
               (origin + dir1 / 2 + dir2 / 2).x,
               (origin + dir1 / 2 + dir2 / 2).y,
               10,
               ColorClasses::yellow,
               (int)(probs[RandomForests::negative] * 100) << "/[" << (int)(probs[RandomForests::line] * 100) << "]");

    } else {
      DRAWTEXT("module:ObjectPerceptor:probs",
               (origin + dir1 / 2 + dir2 / 2).x,
               (origin + dir1 / 2 + dir2 / 2).y,
               10,
               ColorClasses::yellow,
               "[" << (int)(probs[RandomForests::negative] * 100) << "]/" << (int)(probs[RandomForests::line] * 100));
    }
  }
}

void ObjectPerceptor::update(BallPercept& ballPercept) {
  DECLARE_DEBUG_DRAWING("module:ObjectPerceptor:patches", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ObjectPerceptor:probs", "drawingOnImage");

  labelBallSpotsOnce();

  std::vector<const BallSpot*> balls;
  for (const auto& i : ballSpotLabels) {
    if (i.second == RandomForests::ball) {
      balls.push_back(i.first);
    }
  }

  if (balls.empty()) {
    ballPercept.ballWasSeen = false;

  } else {
    auto compareBallProb = [this](const BallSpot* a, const BallSpot* b) {
      return ballSpotProbs.at(a)[RandomForests::ball] < ballSpotProbs.at(b)[RandomForests::ball];
    };
    const BallSpot& bestSpot = **std::max_element(balls.begin(), balls.end(), compareBallProb);
    // saveSpotToFile(bestSpot, std::string(File::getBHDir()) + "/Images/BallPercepts");

    ballPercept.positionInImage = Vector2<>(bestSpot.position.x, bestSpot.position.y);
    ballPercept.radiusInImage = static_cast<float>(bestSpot.radius);
    ballPercept.ballWasSeen = true;

    Vector2<> ballBottomOnImage = theImageCoordinateSystem.toCorrected(
      Vector2<>((float)bestSpot.position.x, (float)(bestSpot.position.y + ballPercept.radiusInImage)));
    Geometry::calculatePointOnField(ballBottomOnImage, theCameraMatrix, theCameraInfo, ballPercept.relativePositionOnField);

    ballPercept.timeWhenLastSeen = static_cast<float>(theFrameInfo.time);

    if (useBallOutsideFieldCheck && (theRobotPose.validity > 0.3)) {
      unseeBallOutsideOfField(ballPercept);
    }
  }
}

void ObjectPerceptor::unseeBallOutsideOfField(BallPercept& ballPercept) {
  Vector2<> globalBallPos = theRobotPose * ballPercept.relativePositionOnField;

  // xAxis check
  if ((globalBallPos.x > (theFieldDimensions.xPosOpponentGroundline + ballOutsideFieldOffset)) ||
      (globalBallPos.x < (-1.f * theFieldDimensions.xPosOpponentGroundline - ballOutsideFieldOffset))) {
    ballPercept.ballWasSeen = false;
  }

  // yAxis check
  if ((globalBallPos.y > (theFieldDimensions.yPosLeftSideline + ballOutsideFieldOffset)) ||
      (globalBallPos.y < (-1.f * theFieldDimensions.yPosLeftSideline - ballOutsideFieldOffset))) {
    ballPercept.ballWasSeen = false;
  }
}

cv::Mat ObjectPerceptor::squeezeImage(
  const Image* image, const cv::Point2f& origin, const cv::Point2f& dir1, const cv::Point2f& dir2, int patchSize) {
  cv::Mat mat(patchSize, patchSize, CV_8UC3);

  const float normalize = 1.f / patchSize;

  for (int i = 0; i < patchSize; ++i) { // row (mat.y / dir2)
    uchar* matRow = mat.ptr(i);
    for (int j = 0; j < patchSize; ++j) { // col (mat.x / dir1)
      const cv::Point2f pt = origin + (dir1 * j + dir2 * i) * normalize;
      const int x = int(std::round(pt.x));
      const int y = int(std::round(pt.y));

      if (x >= 0 && x < image->width && y >= 0 && y < image->height) {
        const Image::Pixel& px = (*image)[y][x];
        matRow[3 * j + 0] = px.y;
        matRow[3 * j + 1] = px.cr;
        matRow[3 * j + 2] = px.cb;
      } else {
        matRow[3 * j + 0] = 0;
        matRow[3 * j + 1] = 128;
        matRow[3 * j + 2] = 0;
      }
    }
  }

  return mat;
}

void ObjectPerceptor::saveSpotToFile(const BallSpot& spot, const std::string& dir) {
  const int patchSize = 24;
  const float marginFactor = 0.0f; // Extra margins that are 25% the radius (approx sqrt(2*pi)/2 - 1)

  const int cr = (int)std::round(spot.radius * (1.0f + marginFactor));
  const int cx = std::max(cr, std::min(spot.position.x, theImage.width - 1 - cr));
  const int cy = std::max(cr, std::min(spot.position.y, theImage.height - 1 - cr));

  const int x1 = cx - cr;
  const int x2 = cx + cr;
  const int y1 = cy - cr;
  const int y2 = cy + cr;
  if (x1 < 0 || x2 >= theImage.width || y1 < 0 || y2 >= theImage.height) {
    return;
  }

  cv::Mat cropped = cropImage(theImage, cv::Rect(x1, y1, x2 - x1, y2 - y1));

  cv::Mat resized;
  cv::resize(cropped, resized, cv::Size(patchSize, patchSize), 0, 0, cv::INTER_NEAREST);

  mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  std::string path = dir + "/" + std::to_string(theFrameInfo.time) + "-" + std::to_string(cr) + "-" + std::to_string(cx) +
                     "-" + std::to_string(cy) + ".bmp";
  cv::imwrite(path, resized);
}

void ObjectPerceptor::update(PlayerPercept& playerPercept) {
  // Declare debug drawing
  DECLARE_DEBUG_DRAWING("module:ObjectPerceptor:patches", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ObjectPerceptor:jerseyScan", "drawingOnImage");

  labelBallSpotsOnce();

  std::vector<const BallSpot*> robots;
  for (const auto& i : ballSpotLabels) {
    if (i.second == RandomForests::robotBody) {
      robots.push_back(i.first);
    }
  }

  playerPercept.players.clear();

  if (!robots.empty()) {
    auto compareXPos = [](const BallSpot* a, const BallSpot* b) { return a->position.x < b->position.x; };
    std::sort(robots.begin(), robots.end(), compareXPos);

    // Cluster patches by image distance
    const int notClustered = -1;
    std::vector<int> membership(robots.size(), notClustered); // one per spot
    int nextCluster = 0;

    for (int i = 0, n = robots.size(); i < n; ++i) {
      const BallSpot* spot = robots[i];
      int cluster = membership[i];

      if (cluster == notClustered) {
        cluster = nextCluster;
        ++nextCluster;

        membership[i] = cluster;
      }

      // Add new members to the cluster
      for (int j = i + 1; j < n; ++j) {
        const BallSpot* spot2 = robots[j];

        if (spot2->position.x - spot->position.x > playerClusterMaxGap) {
          break;
        }
        if (membership[j] == notClustered) {
          membership[j] = cluster;
        }
      }
    }

    const int numClusters = nextCluster;

    // Group spots by their cluster
    std::vector<std::vector<const BallSpot*>> clusters(numClusters);
    for (int i = 0, n = robots.size(); i < n; ++i) {
      clusters[membership[i]].push_back(robots[i]);
    }

    // Make a playerPercept out of each cluster with [param] spots or more
    for (const auto& cluster : clusters) {
      if (cluster.size() < playerClusterMinSpots) {
        continue;
      }

      PlayerPercept::Player player;
      player.x1 = theImage.width;
      player.x2 = 0;
      player.y1 = theImage.height;
      player.y2 = 0;

      // Expand player to bounding box of all spots in cluster (except for top bound)
      for (const BallSpot* spot : cluster) {
        player.x1 = std::min(static_cast<int>(player.x1), spot->position.x - spot->radius);
        player.x2 = std::max(static_cast<int>(player.x2), spot->position.x + spot->radius);
        player.y1 = std::min(static_cast<int>(player.y1), spot->position.y - spot->radius);
        player.y2 = std::max(static_cast<int>(player.y2), spot->position.y + spot->radius);
      }

      Vector2<> leftInImage = theImageCoordinateSystem.toCorrected(Vector2<>(player.x1, player.y2));
      Vector2<> rightInImage = theImageCoordinateSystem.toCorrected(Vector2<>(player.x2, player.y2));
      Vector2<> leftOnField;
      Vector2<> rightOnField;
      Geometry::calculatePointOnField(
        Vector2<int>(leftInImage.x, leftInImage.y), theCameraMatrix, theCameraInfo, leftOnField);
      Geometry::calculatePointOnField(
        Vector2<int>(rightInImage.x, rightInImage.y), theCameraMatrix, theCameraInfo, rightOnField);
      player.centerOnField = (leftOnField + rightOnField) * 0.5f;
      player.radius = (leftOnField - rightOnField).abs();

      player.standing = (player.x2 - player.x1 < playerFallenMinWidth);

      if (theCameraInfo.camera == CameraInfo::upper) {
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
      }

      // Find out whether opponent or not depending on jersey color
      player.detectedJersey = false;
      player.opponent = true;

      if (applyJerseyScan) {
        ColorClasses::Color opponentPlotColor = ColorClasses::red;

        int centerPosImageX = int(player.x2 - (player.x2 - player.x1) / 2);

        float ownJerseyCounter = 0;
        float numOfPx = 0;

        int scanIterY = player.jerseyY1;
        while (scanIterY < player.jerseyY0) {
          // CROSS("module:ObjectPerceptor:jerseyScan", centerPosImageX, scanIterY, 1, 1, Drawings::ps_solid,
          // ColorClasses::black);
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

        // std::cout << "ownJerseyRatio: " << ownJerseyRatio << std::endl;

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

      playerPercept.players.push_back(player);
    }
  }
}
