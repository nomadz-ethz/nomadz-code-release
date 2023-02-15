/**
 * @file LineLocator.h
 *
 * Looks for interesting field line features & offers hypotheses of poses
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/LineLocalization.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/LineAnalysis.h"
#include "Representations/Perception/PenaltyMarkPercept.h"
#include "Core/Debugging/DebugImages.h"

MODULE(LineLocator)
REQUIRES(BallPercept)
REQUIRES(CameraInfo)
REQUIRES(CameraMatrix)
REQUIRES(GameInfo)
REQUIRES(FieldDimensions)
REQUIRES(LineAnalysis)
REQUIRES(OwnTeamInfo)
REQUIRES(PenaltyMarkPercept)
REQUIRES(RobotInfo)
USES(BehaviorControlOutput) // Just need the role
PROVIDES_WITH_MODIFY(LineLocalization)
DEFINES_PARAMETER(float,
                  maxDist,
                  3500.f) // (mm) Maximum allowed distance for a hypothesis from the feature that generated it
DEFINES_PARAMETER(float,
                  outOfFieldMargin,
                  300.f) // (mm) Maximum allowed distance out of the field before a hypothesis is rejected
END_MODULE

class LineLocator : public LineLocatorBase {
public:
  LineLocator() {}

  void init();

  void update(LineLocalization& lineLocalization);

private:
  CameraInfo::Camera lastCamera = CameraInfo::upper;

  std::vector<LineLocalization::Line> lines; // All lines except the middle line
  std::vector<LineLocalization::Intersection> corners;
  std::vector<LineLocalization::Intersection> tees;
  std::vector<LineLocalization::Intersection> crosses;
  std::vector<LineLocalization::PenaltyAreaSide> penaltyAreaSides;

  bool ballSeenLower;
  bool ballSeenUpper;
  Vector2<> ballPosLower;
  Vector2<> ballPosUpper;

  bool markSeenLower;
  bool markSeenUpper;
  Vector2<> markPosLower;
  Vector2<> markPosUpper;

  bool isOutOfField(const Pose2D& pose, float margin) const;

  static void drawPose(const Pose2D& pose, ColorRGBA color);
};
