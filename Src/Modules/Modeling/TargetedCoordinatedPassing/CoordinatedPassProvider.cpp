/**
 * @file CoordinatedPassProvider.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "CoordinatedPassProvider.h"
MAKE_MODULE(CoordinatedPassProvider, Modeling)

CoordinatedPassProvider::CoordinatedPassProvider() {
  provider = 0;
}

/**
 * Provides the coordinated Pass representation
 */
void CoordinatedPassProvider::update(CoordinatedPassRepresentation& coordinatedPassRepresentation) {
  MODIFY("module:CoordinatedPassProvider:CoordinatedPassParameters", parameters);

  // Check if a new target point is wanted are wanted (otherwise just wait)
  if (thePersonalData.needToCalculatePass == true) {
    coordinatedPassRepresentation.passCoordinate = theFieldDimensions.randomPoseOnCarpet().translation;
    coordinatedPassRepresentation.distancePass = (theCombinedWorldModel.ballState.position - theRobotPose.translation).abs();
    coordinatedPassRepresentation.foundPassTarget = true;
    passCoordinate = coordinatedPassRepresentation.passCoordinate;
    std::cout << theRobotInfo.number << "  x: " << coordinatedPassRepresentation.passCoordinate.x
              << "   y: " << coordinatedPassRepresentation.passCoordinate.y << std::endl;
  } else {
    coordinatedPassRepresentation.foundPassTarget = false;
  }

  drawPassInfo();
  // std::cout << theRobotInfo.number << "  " << parameters.flag << std::endl;
}

/* Draw:
 * - the desired collision point between the foot and ball
 * - the targeted position
 * - a line which indicates the distance at which the robot should start the kick motion
 *
 *  */
void CoordinatedPassProvider::drawPassInfo() {
  DECLARE_DEBUG_DRAWING("module:CoordinatedPassProvider:drawTargetPosition", "drawingOnField");

  // Draw the targeted point onto the field

  CIRCLE("module:CoordinatedPassProvider:drawTargetPosition",
         passCoordinate.x,
         passCoordinate.y,
         200,
         10,
         Drawings::PenStyle::ps_solid,
         colorRed,
         Drawings::FillStyle::bs_solid,
         colorWhite);
  CIRCLE("module:CoordinatedPassProvider:drawTargetPosition",
         passCoordinate.x,
         passCoordinate.y,
         150,
         10,
         Drawings::PenStyle::ps_solid,
         colorRed,
         Drawings::FillStyle::bs_solid,
         colorNone);
  CIRCLE("module:CoordinatedPassProvider:drawTargetPosition",
         passCoordinate.x,
         passCoordinate.y,
         100,
         10,
         Drawings::PenStyle::ps_solid,
         colorRed,
         Drawings::FillStyle::bs_solid,
         colorNone);
  LARGE_DOT("module:CoordinatedPassProvider:drawTargetPosition",
            passCoordinate.x,
            passCoordinate.y,
            ColorClasses::red,
            ColorClasses::red);
}