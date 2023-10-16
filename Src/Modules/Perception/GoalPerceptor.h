/**
 * @file GoalPerceptor.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are Michel Bartsch and Thomas Münder
 */

#pragma once

#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"
#include "Core/Module/Module.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Representations/Perception/ColorReference.h"
#include "Representations/BehaviorControl/GoalSymbols.h"

MODULE(GoalPerceptor)
REQUIRES(CameraMatrix)
REQUIRES(ImageCoordinateSystem)
REQUIRES(CameraInfo)
REQUIRES(Image)
REQUIRES(FieldDimensions)
REQUIRES(FrameInfo)
REQUIRES(ColorReference)
REQUIRES(FieldBoundary)
REQUIRES(Odometer)
PROVIDES_WITH_MODIFY_AND_DRAW(GoalPercept)
PROVIDES(GoalSymbols)
LOADS_PARAMETER(int, quality)
LOADS_PARAMETER(int, yellowSkipping)
END_MODULE

/**
 * @class GoalPerceptor
 */
class GoalPerceptor : public GoalPerceptorBase {
private:
  struct Spot {
  public:
    Spot(int s, int e, int h) : start(s), end(e), validity(100) {
      width = end - start;
      mid = Vector2<int>(start + (width / 2), h);
    }

    inline bool operator<(const Spot& other) const { return validity < other.validity; }

    int start;
    int end;
    int width;
    std::vector<int> widths;
    Vector2<int> mid;
    Vector2<int> base;
    Vector2<int> top;
    GoalPost::Position leftRight;
    Vector2<> position;
    float validity;
  };

  void update(GoalPercept& percept);

  void update(GoalSymbols& symbols);

  void findSpots(const int& height);

  void verticalColorScanDown();

  void verticalColorScanUp();

  bool isWhite(const int& x, const int& y);

  void calculatePosition(const int& height);

  void validate();

  void posting(GoalPercept& percept);

  std::list<Spot> spots;

  std::vector<Spot> lastPosts;
};
