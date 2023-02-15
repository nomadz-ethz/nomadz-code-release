/**
 * @file ObjectPerceptor.h
 *
 * This file declares a module which updates the following representations:
 *      - BallPercept -> For ball detection
 *      - LinePercept -> For line detection
 *      - PlayerPercept -> For player detection
 * It is using priors from other representations and the random forest classifiers
 * to verify them
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
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/PlayerPercept.h"
#include "Representations/Perception/ShapePercept.h"
#include "Representations/Perception/RandomForests.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Modeling/RobotPose.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Tools/RandomForest/Tree.h"
#include "Tools/RandomForest/StatisticFunctions.h"

MODULE(ObjectPerceptor)

REQUIRES(Image)
REQUIRES(FrameInfo)
REQUIRES(CameraInfo)
REQUIRES(CameraMatrix)
REQUIRES(FieldDimensions)
REQUIRES(ShapePercept)
REQUIRES(BallSpots)
REQUIRES(RandomForests)
REQUIRES(ImageCoordinateSystem)
REQUIRES(OwnTeamInfo)
USES(RobotPose)
PROVIDES_WITH_MODIFY_AND_OUTPUT(LinePercept)
PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallPercept)
PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(PlayerPercept)
LOADS_PARAMETER(unsigned int, playerClusterMinSpots)
LOADS_PARAMETER(float, playerClusterMaxGap)
LOADS_PARAMETER(float, playerFallenMinWidth)
LOADS_PARAMETER(std::vector<float>, playerJerseyY)
LOADS_PARAMETER(bool, useBallOutsideFieldCheck)
LOADS_PARAMETER(float, ballOutsideFieldOffset)
LOADS_PARAMETER(bool, excludeRobotsOnBall)
LOADS_PARAMETER(bool, applyJerseyScan)
LOADS_PARAMETER(float, ownJerseyThreshold)
LOADS_PARAMETER(int, redJerseyRGBThreshold)
LOADS_PARAMETER(int, blueJerseyRGBThreshold)

END_MODULE

class ObjectPerceptor : public ObjectPerceptorBase {

private:
  cv::Mat cropImage(const Image&, const cv::Rect&);
  void saveSpotToFile(const BallSpot&, const std::string&);

  unsigned int lastLabeled = 0;
  std::map<const BallSpot*, std::map<RandomForestLabel, float>> ballSpotProbs;
  std::map<const BallSpot*, RandomForestLabel> ballSpotLabels;
  void labelBallSpotsOnce();

  void unseeBallOutsideOfField(BallPercept& ballPercept);

  static cv::Mat squeezeImage(
    const Image* image, const cv::Point2f& origin, const cv::Point2f& dir1, const cv::Point2f& dir2, int patchSize);

public:
  ObjectPerceptor() {}  // Constructor
  ~ObjectPerceptor() {} // Destructor

  // Update functions
  void update(LinePercept& linePercept);
  void update(BallPercept& ballPercept);
  void update(PlayerPercept& playerPercept);
};
