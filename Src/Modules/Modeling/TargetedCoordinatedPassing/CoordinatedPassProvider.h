/**
 * @file CoordinatedPassProvider.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Math/Vector2.h"
#include "Core/Streams/OutStreams.h"
#include "Core/Streams/InOut.h"

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/FootContactModel.h"
#include "Representations/Modeling/CoordinatedPassRepresentation.h"

#include "CoordinatedPassParameters.h"

#include <iostream>

MODULE(CoordinatedPassProvider)
REQUIRES(RobotPose)
REQUIRES(TeamMateData)
REQUIRES(RobotInfo)
REQUIRES(FrameInfo)
REQUIRES(BallModel)
PROVIDES_WITH_MODIFY(CoordinatedPassRepresentation)
USES(FieldDimensions)
USES(PersonalData)
USES(CombinedWorldModel)
END_MODULE

/**
 * @class CoordinatedPassProvider
 */
class CoordinatedPassProvider : public CoordinatedPassProviderBase {
private:
  int provider;

  // ============================= CoordinatedPassProvider element needed for calculation
  //
  // USER DEFINED: All the user parameters
  CoordinatedPassParameters parameters;

  // Flag that tell us if we found the pass point or not
  bool drawPassPoint;

  // Variable to store and draw the coordinates of the pass target
  Vector2<> passCoordinate;

  // Definition of the Debug drawing colors
  ColorRGBA colorWhite = ColorRGBA(255, 255, 255, 90);
  ColorRGBA colorRed = ColorRGBA(255, 0, 0, 150);
  ColorRGBA colorNone = ColorRGBA(0, 0, 0, 0);

  /**
   * Provides ball model representation
   */
  void update(CoordinatedPassRepresentation& coordinatedPassRepresentation);

  // ============================= Methods - Drawing (all done in worldstate)

  // Function that contains all the drawings
  void draw();

  // Draw the targeted position into the field
  void drawPassInfo();

public:
  /**
   * Default constructor.
   */
  CoordinatedPassProvider();
};
