/**
 * @file PassHelperProvider.h
 *
 * PassHelperProvider is a Module that implement all the method needed for selecting the pass point
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "PassTypes.h"
#include "PassTypeDef.h"

#include "PassHelperProviderParameters.h"
#include "PointsField.h"
#include "DrawingOptions.h"
#include "LegendOptions.h"

#include "Representations/Modeling/PassHelper.h"

#include "Core/Module/Module.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Modeling/RobotPose.h"

#include "Core/Debugging/DebugDrawings.h"

#include "Core/Streams/OutStreams.h"
#include "Core/Streams/InOut.h"

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

// Declare the module and his dependencies
MODULE(PassHelperProvider)
REQUIRES(RobotPose)
REQUIRES(TeamMateData)
REQUIRES(BallModel)
USES(PersonalData)
USES(CombinedWorldModel)
REQUIRES(RobotInfo)
USES(FieldDimensions)
PROVIDES_WITH_MODIFY(PassHelper)
END_MODULE

// Note:

// Requires:
// The class collects all requirements of a certain module, i.e. the representations
// that have to be updated before the module is executed.

// Uses:
// The macro defines a usage, i.e. a representation that is accessed but does not need to be up to date.

// Provides with modify:
// The macro defines a representation that is updated by this module.

class PassHelperProvider : public PassHelperProviderBase {

public:
  PassHelperProvider();

private:
  // ============================= PassHelperProvider element needed for calculation

  // USER DEFINED: All the user tuned parameters
  PassHelperProviderParameters parameters;

  // USER DEFINED: Define the areas on the field which belong to some pass type
  PassZones passZones;

  // Create the field and store all the points in pointsPtr
  PointsField points;

  // Actual points used
  LocTarget** pointsPtr;

  // Note: Access the structure that pointsPtr is point to as

  // pointsPtr[0][0] ------------> pointsPtr[0][m_column -1]
  //   :						  :
  //   :						  :
  // pointsPtr[m_row-1][0] ------> pointsPtr[m_row-1][m_column -1]

  // These index correspond to this configurations

  //  pointsPtr[0][0]  ^ +y
  //  	  	  	  	 |
  //  --------------------------------> +x
  //  	  	  	  	 |  pointsPtr[m_row-1][m_column -1]

  // Vector containing the index of locations where we could pass the ball, i.e. the point contained in a square around the
  // player
  std::vector<std::vector<Index>> possiblePointsIndex;

  // Vector containing the index of all the points that are valid for a pass
  std::vector<std::vector<Index>> selectedPointsIndex;

  // Tell if the calculation of the module should be done or not (on real robot needed to spare computer power)
  bool calculation;

  // ============================= Representation

  // Final position of the pass
  Location passLocation;

  // Distance from the pass player to the passing point, needed for the strength calculation of the shoot
  float distancePass;

  // Flag that tell us if we found the pass point or not
  bool foundPassPoint;

  // Coordinate of the pass point
  Location passCoordinate;

  // Id (number) of the receiving robot
  int idRecieverPass;

  // Type of pass that we are going to perform
  PassHelper::TypePass typePass;

  // ============================= Drawing element

  // Standard color used for the drawings
  Color standardColor;

  // USER DEFINED: Class of boolean for selecting which drawing we want to see in worldstate
  DrawingOptions drawingOptions;

  // USER DEFINED: Standard option used for most of the legend in the drawing of worldstate
  LegendOptions legendOptions;

  // This vector is only needed for the shadows drawing
  std::vector<drawingShadowPoints> PointTangentCirlce;

  // ============================= Methods - Algorithm

  // This function is called at every iteration of the robot controller, and update the representation of this module =
  // PassHelper
  void update(PassHelper& passHelper);

  // This function find all the possible points around a robot that could be used as pass location
  void findPossiblePoints();

  // Given the location of one robot (center), this function calculate the shadow that this robot do. With shadow is to
  // intend all the points that we don't
  // want to consider for the pass, since if we would pass the ball there, we would hit the robot instead to reach the
  // desired locations
  void calculateSingleShadow(Point2D center);

  // Calculate the shadow of every robots (team mate and opponent)
  void calculateTotalShadow();

  // Set the value of every points of the Field as false
  void setAllPointsFalse();

  // Set the value of every points of the Field as true
  void setAllPointsTrue();

  // Decide which is the Point where we will perform the pass, which player is the target and which kind of pass will be
  // performed
  void decidePassPoint();

  // We take all the index of the possible points, and we look according to the shadows if they are still valid or not. If
  // they are, they will be stored in the
  // selectedPointsIndex vector
  void evaluatePossiblePoints();

  // Calculate if the Point is inside the valid range of the front kick (forward + backward)
  bool insidePassFrontRange(Index Point);

  // Calculate if the Point is inside the valid range of the lateral kick
  bool insidePassLateralRange(Index Point);

  // Decide the best point for the forward pass (if it exist), and store it in passLocation
  void decidePassPointForward();

  // Decide the best point for the backward pass (if it exist), and store it in passLocation
  void decidePassPointBackward();

  // Decide the best point for the lateral pass (if it exist), and store it in passLocation
  void decidePassPointLateral();

  // Return the distance between the robot and a Point
  float robotDistanceToPoint(Index Point);

  // Return the distance between the ball and a Point
  float ballDistanceToPoint(Index Point);

  // Set all the points in the vector possiblePointsIndex to true
  void setPossibleTargetTrue();

  // ============================= Methods - Drawing (all done in worldstate)

  // Function that contains all the drawings
  void draw();

  // Draw all the shadows of the other robots and opponent (this include also the 4 goal post
  void drawTotalShadow();

  // Draw at the location of every robot his own id / number
  void drawAllRobot();

  // Draw these points
  void drawPossiblePoints();

  // Draw the point that are valid for the pass / if the flag AllEvaluatedTargetPoints is also set to true draw also the
  // points discarded by shadow
  void drawEvaluatedPoints();

  // Draw the final point chosen for the pass
  void drawPassPoint();

  // Draw the pass zones, separated according to the type of pass that we want to perform
  void drawPassZone();
};