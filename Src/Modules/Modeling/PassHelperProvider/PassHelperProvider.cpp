/**
 * @file PassHelperProvider.cpp
 *
 * Implementation of the methods of the class PassHelperProvider
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "PassHelperProvider.h"

// Generate the module
MAKE_MODULE(PassHelperProvider, Modeling)
PassHelperProvider::PassHelperProvider()
    : points(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline),
      legendOptions(static_cast<int>(theFieldDimensions.xPosOpponentFieldBorder),
                    static_cast<int>(theFieldDimensions.yPosLeftFieldBorder)),
      standardColor(), passZones(theFieldDimensions, parameters), possiblePointsIndex(theTeamMateData.numOfPlayers),
      selectedPointsIndex(theTeamMateData.numOfPlayers) {

  // Decide the dimension of the the array, i.e. the number of point that we are going to use
  points.decideNumberOfPoints();

  points.createPoints(pointsPtr);

  // populate the map
  points.populate(pointsPtr);
}

void PassHelperProvider::update(PassHelper& passHelper) {
  // Enable options for the different drawings with vd module:PassHelperProvider:drawingOptions
  MODIFY("module:PassHelperProvider:drawingOptions", drawingOptions);
  MODIFY("module:PassHelperProvider:legendOptions", legendOptions);

  // Check if the continuous calculation are wanted, else set a flag to know when we start with the calculation
  if (parameters.continuousCalculation) {
    calculation = true;
  } else {
    // Check if the pass skills tell us that the calculation are needed
    if (thePersonalData.needToCalculatePass) {
      calculation = true;
    } else {
      calculation = false;
    }
  }

  // Note: If the module has to calculate or not is not decided by the module but from the player. This is done since maybe
  // in the future other role rather than
  // striker will need to make these calculation and so the module is independent of his user

  // Now for the striker: There is a range where the module will calculate the pass point. As soon as it is enough close to
  // the ball, the striker will send via
  // thePersonalData.needToCalculatePass that the calculation should stop, and it will take the last calculated value as the
  // final destination of the pass.
  // In this way the robot will not change his mind everytime, and it will perform the action.

  if (calculation) {
    // Choose the possible candidates to the pass
    findPossiblePoints();

    // Calculate the shadows of all the robot on the field
    calculateTotalShadow();

    // Store the points that are not in the shadow in the selectedPointsIndex vector
    evaluatePossiblePoints();

    // Decide according to where we are which kind of pass are we going to perform and call the appropriate function
    decidePassPoint();

    // Now we need to update the representation with what we found with the calculations
    passHelper.foundPassPoint = foundPassPoint;
    passHelper.passCoordinate = passCoordinate;
    passHelper.distancePass = distancePass;
    passHelper.idRecieverPass = idRecieverPass;
    passHelper.typePass = typePass;
  }

  // Draw all the debugging drawing in worldstate. This is put outside the if statement since if we want drawing even if the
  // flag parameters.continuousCalculation
  // is set to false we always have to call the Draw Function
  draw();
}

void PassHelperProvider::draw() {
  // Draw in worldstate all the robots number at their location
  if (drawingOptions.AllRobotNumbers) {
    drawAllRobot();
  }

  // Draw all the pass Zones
  if (drawingOptions.PassZones) {
    drawPassZone();
  }

  // Draw all the points on the field
  if (drawingOptions.AllPoints) {
    points.drawAllPoints(pointsPtr);
  }

  // Draw the output
  if (drawingOptions.PossiblePoints) {
    drawPossiblePoints();
  }

  // Draw the Shadows
  if (drawingOptions.Shadows) {
    drawTotalShadow();
  }

  // Draw the previous possible target points with the influence of the shadows (i.e. if these points are still valid or not)
  if (drawingOptions.SelectedPoints) {
    drawEvaluatedPoints();
  }

  // Draw this point
  if (drawingOptions.PassPoint) {
    drawPassPoint();
  }

  // This unfortunately doesn't work.. no idea why
  if (drawingOptions.SetAllFalse) {
    //		drawingOptions.SetAllFalse = false;

    // set all drawing as false
    drawingOptions.AllRobotNumbers = false;
    drawingOptions.AllPoints = false;
    drawingOptions.PossiblePoints = false;
    drawingOptions.Shadows = false;
    drawingOptions.SelectedPoints = false;
    drawingOptions.AllSelectedPoints = false;
    drawingOptions.PassPoint = false;
    drawingOptions.PassZones = false;
    drawingOptions.VisualizeLegend = false;
  }
}

void PassHelperProvider::findPossiblePoints() {
  // Clear the vector from the previous data
  for (int i = 0; i < theTeamMateData.numOfPlayers; i++) {
    possiblePointsIndex[i].clear();
  }

  for (int i = 0; i < theTeamMateData.numOfPlayers; i++) {
    // If the robot from which we want to calculate the shadow is the robot itself, just skip
    // No need to make calculation for the robot which will pass the ball
    if (i == theRobotInfo.number) {
      continue;
    }

    if (theTeamMateData.isFullyActive[i]) {
      // Find out which is the corresponding index of the robot
      Index robotIndex = points.findIndexRobot(theTeamMateData.robotPoses[i].translation);

      // The -1 index is given if the robot is outside the field
      // This is done by the function findIndexRobot in PointsField class
      if (robotIndex.first == -1 || robotIndex.second == -1) {
        continue;
      }

      int robotRow = robotIndex.first;
      int robotColumn = robotIndex.second;

      possiblePointsIndex[i].push_back(robotIndex);

      // Calculate all the possible index around the robot
      for (int row = -parameters.squareHalfDimension; row <= parameters.squareHalfDimension; row++) {
        for (int column = -parameters.squareHalfDimension; column <= parameters.squareHalfDimension; column++) {
          int row_tmp, column_tmp;
          row_tmp = robotRow + row;
          column_tmp = robotColumn + column;

          if ((0 <= row_tmp) && (row_tmp < points.n_row)) {
            if ((0 <= column_tmp) && (column_tmp < points.n_column)) {
              possiblePointsIndex[i].push_back(std::make_pair(row_tmp, column_tmp));
            }
          }
        }
      }
    }
  }
}

void PassHelperProvider::drawPossiblePoints() {

  DECLARE_DEBUG_DRAWING("module:PassHelperProvider:drawPossibleTargetPoints", "drawingOnField");

  // Use this function to initialize the Legend
  legendOptions.startLegend();

  ColorRGBA color;
  unsigned char value;
  int diff = 40;
  for (int i = 0; i < theTeamMateData.numOfPlayers; i++) {
    // Here we don't need to check weather the robot is active or the passing player since this was already done in the
    // previous step (findPossiblePoints)
    // In the case the robot is not active or the passing player, the array corresponding will be empty
    if (!possiblePointsIndex[i].empty()) {
      // Change the intensity of the gray according to the player id
      value = 230 - i * diff;

      color = ColorRGBA(value, value, value);

      for (std::vector<Index>::iterator it = possiblePointsIndex[i].begin(); it != possiblePointsIndex[i].end(); ++it) {
        LARGE_DOT("module:PassHelperProvider:drawPossibleTargetPoints",
                  pointsPtr[it->first][it->second].first.x,
                  pointsPtr[it->first][it->second].first.y,
                  ColorClasses::black,
                  color);
      }

      // If we want the legend
      if (drawingOptions.VisualizeLegend) {
        // Draw the Legend
        FILLED_RECTANGLE("module:PassHelperProvider:drawPossibleTargetPoints",
                         legendOptions.sLowerLeftX,
                         legendOptions.textY,
                         legendOptions.sUpperRightX,
                         legendOptions.sUpperRightY,
                         0,
                         Drawings::PenStyle::ps_solid,
                         standardColor.penColor,
                         Drawings::FillStyle::bs_solid,
                         color);

        DRAWTEXT("module:PassHelperProvider:drawPossibleTargetPoints",
                 legendOptions.textX,
                 legendOptions.textY,
                 legendOptions.Font,
                 ColorClasses::black,
                 "Robot " << i);

        legendOptions.nextLine();
      }
    }
  }
}

void PassHelperProvider::setAllPointsFalse() {
  for (int row = 0; row < points.n_row; row++) {
    for (int column = 0; column < points.n_column; column++) {
      pointsPtr[row][column].second = false;
    }
  }
}

void PassHelperProvider::setAllPointsTrue() {
  for (int row = 0; row < points.n_row; row++) {
    for (int column = 0; column < points.n_column; column++) {
      pointsPtr[row][column].second = true;
    }
  }
}

void PassHelperProvider::setPossibleTargetTrue() {
  for (int i = 0; i < theTeamMateData.numOfPlayers; i++) {
    for (std::vector<Index>::iterator it = possiblePointsIndex[i].begin(); it != possiblePointsIndex[i].end(); ++it) {
      pointsPtr[it->first][it->second].second = true;
    }
  }
}

void PassHelperProvider::calculateTotalShadow() {
  setAllPointsTrue();
  PointTangentCirlce.clear();

  // First calculate the shadow for our team
  for (int i = 0; i < theTeamMateData.numOfPlayers; i++) {
    // Notice: here I need to check whether the player considered is the pass player and if not if the player is active since
    // we are not considering
    // the possibleIndex vector, where these condition are already fulfilled

    // If the robot from which we want to calculate the shadow is the robot itself, just skip
    // No need to make calculation for the robot which will pass the ball
    if (i == theRobotInfo.number) {
      continue;
    }

    // If a robot is not active, then we don't need to calculate the shadow
    if (!theTeamMateData.isActive[i]) {
      continue;
    }

    calculateSingleShadow(theTeamMateData.robotPoses[i].translation);
  }

  for (std::vector<GaussianPositionDistribution>::const_iterator it = theCombinedWorldModel.positionsOpponentTeam.begin();
       it != theCombinedWorldModel.positionsOpponentTeam.end();
       ++it) {
    // Since we don't want to pass the ball to much near the goal post, we don't avoid to count them as an obstacle
    // In fact usually the goal post is included in the opponents
    calculateSingleShadow(it->robotPosition);
  }
}

void PassHelperProvider::calculateSingleShadow(Point2D center) {

  /******** Calculate the line parameters ********/
  Point2D externalPoint, pointUp, pointDown;

  // Here we distinguish between Point2D and Vector2 even if this is the same since Point2D is to represent as a point,
  // rather Vector2 as a direction
  Vector2<> Delta_externalPoin_center, upDist, downDist;
  float externalPoint_center_norm;

  // The angle between the two tangents in externalPoint is 2*alpha
  float sinAlpha;
  float alpha;

  // Angle between x-Axe and externalPoint_center
  float gamma;

  // For calculation
  float angle;

  // Location of the ball
  externalPoint = theCombinedWorldModel.ballState.position;

  // Find alpha
  externalPoint_center_norm = (center - externalPoint).abs();
  sinAlpha = parameters.robotShadowRadius / externalPoint_center_norm;
  alpha = asin(sinAlpha);

  // Find gamma
  Delta_externalPoin_center = center - externalPoint;
  gamma = Delta_externalPoin_center.angle();

  // Calculate pointUp
  angle = alpha + gamma;
  upDist = Vector2<>(cos(angle), sin(angle));
  pointUp = externalPoint + upDist.normalize(externalPoint_center_norm);

  // Calculate pointDown
  angle = gamma - alpha;
  downDist = Vector2<>(cos(angle), sin(angle));
  pointDown = externalPoint + downDist.normalize(externalPoint_center_norm);

  // These two instance contain the slope (m) and the y intercept (q)
  StraightLine<> slUp(externalPoint, pointUp);
  StraightLine<> slDown(externalPoint, pointDown);

  // Just for the debug drawing
  PointTangentCirlce.push_back({center, pointUp, pointDown});

  /******** Calculate if a point is reachable or not ********/
  float x, y;
  float dX, dY;
  bool keep;
  float yUp, yDown;
  double dist;
  float absDifference;

  // Since all the points are true, every point in the shadow should be denoted as false
  for (int row = 0; row < points.n_row; row++) {
    for (int column = 0; column < points.n_column; column++) {
      // If a point is already marked as false, then we don't need to check it twice
      // When we will have the whole map with the points that are reachable and not, since we already have the index of the
      // points where we could pass the ball,
      // we are going just to look if at these point the ball could be passed or not.
      if (pointsPtr[row][column].second == false) {
        continue;
      }

      x = pointsPtr[row][column].first.x;
      y = pointsPtr[row][column].first.y;

      // The points between the passing and the receiving robot should not be labeled as false!
      // This could of course be valid in all the direction, but since we are interested in the points shadowed from the
      // receiving robot, if
      // a point is not between the passing and receiving robot, it will however be true (valid for a pass)
      // The formulation here is not exactly correct, but an approximation in order to make the calculation faster
      dist = std::sqrt(pow(x - externalPoint.x, 2) + pow(y - externalPoint.y, 2));
      if (dist < ((center - externalPoint).abs() - parameters.robotShadowRadius)) {
        continue;
      }

      // keep the possible target location or not, depending on the shadow
      keep = true;

      // See documentation for an explanation. Basically given an x, we know for the two bounding lines where the point
      // considered should be in order
      // to keep it or not. The idea is to see if it's y is bounded from yUp and yDown
      yUp = slUp.m * x + slUp.q;
      yDown = slDown.m * x + slDown.q;

      // our goal ------- opponent goal
      // P: pass player
      // R: pass Receiving

      // dX > 0 :	R <---- P
      // dX < 0 :	p ----> R

      //		dY > 0 	:		P						dY < 0  : 	R
      //						|									^
      //						|									|
      //						v									|
      //						R									P

      if (sgn(slUp.m) == sgn(slDown.m)) {
        // This is the case when the circle is represented in the 4 quadrants, but not on the abscissa and
        // ordinates axes
        if (externalPoint.x < center.x) {
          if ((y < yUp) and (y > yDown)) {
            // Discard
            keep = false;
          }
        } else {
          if ((y > yUp) and (y < yDown)) {
            // Discard
            keep = false;
          }
        }

      } else {
        // mUp != mDown
        // This is the case when we are going with the circle through an axes w.r.t. one robot
        dX = externalPoint.x - center.x;
        dY = externalPoint.y - center.y;
        absDifference = std::abs(dY) - std::abs(dX);

        // Now if absDifference > 0 --> we are on the ordinate axes (y)
        //		  absDifference < 0 --> we are on the abscissa axes (x)

        // Quadrants
        //
        //			II		|		I
        //	    --------------------------
        //			III		|		IV
        //

        if (absDifference > 0) // y axe
        {
          //		dY > 0:			P						dY < 0:  	R
          //						|									^
          //						|									|
          //						v									|
          //						R									P
          if (dY > 0) {
            // We are in the II or I quadrant
            if ((y < yUp) and (y < yDown)) {
              // Discard
              keep = false;
            }
          } else {
            // We are in the III or IV quadrant
            if ((y > yUp) and (y > yDown)) {
              // Discard
              keep = false;
            }
          }
        } else // x axe
        {
          if (dX > 0) // R <--- P
          {
            // We are in the I or IV quadrant
            if ((y > yUp) and (y < yDown)) {
              // Discard
              keep = false;
            }
          } else // P ---> R
          {
            // We are in the II or III quadrant
            if ((y < yUp) and (y > yDown)) {
              // Discard
              keep = false;
            }
          }
        }
      }

      // At every iteration we have to be careful that we will not delete the previous value that we found for every points.
      // Therefore we change only if the value is false
      if (!keep) {
        pointsPtr[row][column].second = keep;
      }
    }
  }
}

void PassHelperProvider::drawTotalShadow() {
  DECLARE_DEBUG_DRAWING("module:PassHelperProvider:drawShadow", "drawingOnField");

  ColorClasses::Color color = ColorClasses::Color::white;

  for (std::vector<drawingShadowPoints>::iterator it = PointTangentCirlce.begin(); it != PointTangentCirlce.end(); ++it) {
    //	penWidth, penStyle, penColor, fillStyle, fillColor
    CIRCLE("module:PassHelperProvider:drawShadow",
           it->cirlceCenter.x,
           it->cirlceCenter.y,
           parameters.robotShadowRadius,
           standardColor.penWidth,
           standardColor.penStyle,
           standardColor.penColor,
           standardColor.fillStyle,
           standardColor.fillColor);

    // 	penWidth, penStyle, penColor
    // penColor white for better seen
    LINE("module:PassHelperProvider:drawShadow",
         theCombinedWorldModel.ballState.position.x,
         theCombinedWorldModel.ballState.position.y,
         it->pointUp.x,
         it->pointUp.y,
         30,
         standardColor.penStyle,
         color);

    LINE("module:PassHelperProvider:drawShadow",
         theCombinedWorldModel.ballState.position.x,
         theCombinedWorldModel.ballState.position.y,
         it->pointDown.x,
         it->pointDown.y,
         30,
         standardColor.penStyle,
         color);
  }

  // Draw just the points that are discarded
  for (int row = 0; row < points.n_row; row++) {
    for (int column = 0; column < points.n_column; column++) {
      if (!pointsPtr[row][column].second) {
        // Discared points
        LARGE_DOT("module:PassHelperProvider:drawShadow",
                  pointsPtr[row][column].first.x,
                  pointsPtr[row][column].first.y,
                  standardColor.penColor,
                  color);
      }
    }
  }

  // Used just for report
  //	// Draw the Legend
  //	legendOptions.startLegend();
  //
  //	if(drawingOptions.VisualizeLegend)
  //	{
  //		// Draw the Legend
  //		FILLED_RECTANGLE("module:PassHelperProvider:drawShadow",
  //				legendOptions.sLowerLeftX, legendOptions.textY,
  //				legendOptions.sUpperRightX, legendOptions.sUpperRightY,
  //				0, Drawings::PenStyle::ps_solid, standardColor.penColor, Drawings::FillStyle::bs_solid, ColorClasses::white);
  //
  //		DRAWTEXT("module:PassHelperProvider:drawShadow", legendOptions.textX,
  //				legendOptions.textY, legendOptions.Font, ColorClasses::black, "Points Invalid");
  //
  //		legendOptions.nextLine();
  //
  //		// Draw the Legend
  //		FILLED_RECTANGLE("module:PassHelperProvider:drawShadow",
  //				legendOptions.sLowerLeftX, legendOptions.textY,
  //				legendOptions.sUpperRightX, legendOptions.sUpperRightY,
  //				0, Drawings::PenStyle::ps_solid, standardColor.penColor, Drawings::FillStyle::bs_solid, ColorClasses::black);
  //
  //		DRAWTEXT("module:PassHelperProvider:drawShadow", legendOptions.textX,
  //				legendOptions.textY, legendOptions.Font, ColorClasses::black, "Points Valid");
  //	}
}

void PassHelperProvider::evaluatePossiblePoints() {

  for (int i = 0; i < theTeamMateData.numOfPlayers; i++) {
    // First clear the vector
    selectedPointsIndex[i].clear();

    // If the robot considered is the passing robot, no need to consider it
    if (i == theRobotInfo.number) {
      continue;
    }

    // If a robot is not active, then we don't need to evaluate the points around it
    if (!theTeamMateData.isActive[i]) {
      continue;
    }

    for (std::vector<Index>::iterator it = possiblePointsIndex[i].begin(); it != possiblePointsIndex[i].end(); ++it) {
      if (pointsPtr[it->first][it->second].second) {
        // The point is valid for a pass -> store it in the appropriate vector
        selectedPointsIndex[i].push_back(*it);
      }
    }
  }
}

void PassHelperProvider::drawEvaluatedPoints() {
  DECLARE_DEBUG_DRAWING("module:PassHelperProvider:drawEvaluatedTargetPoints", "drawingOnField");

  ColorClasses::Color color = ColorClasses::Color::red;

  for (int i = 0; i < theTeamMateData.numOfPlayers; i++) {
    // Check if we want to show only the point that are selected or all points around the robots, distinguishing if they are
    // valid or not
    if (drawingOptions.AllSelectedPoints) {
      for (std::vector<Index>::iterator it = possiblePointsIndex[i].begin(); it != possiblePointsIndex[i].end(); ++it) {
        if (!pointsPtr[it->first][it->second].second) {
          // Discard
          color = ColorClasses::yellow;
        } else {
          // Keep
          color = ColorClasses::red;
        }

        LARGE_DOT("module:PassHelperProvider:drawEvaluatedTargetPoints",
                  pointsPtr[it->first][it->second].first.x,
                  pointsPtr[it->first][it->second].first.y,
                  standardColor.penColor,
                  color);
      }
    } else {
      // Just draw the possible target points
      for (std::vector<Index>::iterator it = selectedPointsIndex[i].begin(); it != selectedPointsIndex[i].end(); ++it) {
        LARGE_DOT("module:PassHelperProvider:drawEvaluatedTargetPoints",
                  pointsPtr[it->first][it->second].first.x,
                  pointsPtr[it->first][it->second].first.y,
                  standardColor.penColor,
                  color);
      }
    }

    // Draw the Legend
    legendOptions.startLegend();

    if (drawingOptions.VisualizeLegend) {
      // Draw the Legend
      FILLED_RECTANGLE("module:PassHelperProvider:drawEvaluatedTargetPoints",
                       legendOptions.sLowerLeftX,
                       legendOptions.textY,
                       legendOptions.sUpperRightX,
                       legendOptions.sUpperRightY,
                       0,
                       Drawings::PenStyle::ps_solid,
                       standardColor.penColor,
                       Drawings::FillStyle::bs_solid,
                       ColorClasses::yellow);

      DRAWTEXT("module:PassHelperProvider:drawShadow",
               legendOptions.textX,
               legendOptions.textY,
               legendOptions.Font,
               ColorClasses::black,
               "Points Discarded");

      legendOptions.nextLine();

      // Draw the Legend
      FILLED_RECTANGLE("module:PassHelperProvider:drawEvaluatedTargetPoints",
                       legendOptions.sLowerLeftX,
                       legendOptions.textY,
                       legendOptions.sUpperRightX,
                       legendOptions.sUpperRightY,
                       0,
                       Drawings::PenStyle::ps_solid,
                       standardColor.penColor,
                       Drawings::FillStyle::bs_solid,
                       ColorClasses::red);

      DRAWTEXT("module:PassHelperProvider:drawShadow",
               legendOptions.textX,
               legendOptions.textY,
               legendOptions.Font,
               ColorClasses::black,
               "Points Selected");
    }
  }
}

void PassHelperProvider::drawPassZone() {
  DECLARE_DEBUG_DRAWING("module:PassHelperProvider:drawPassZone", "drawingOnField");

  ColorClasses::Color fillColor;

  /*********** Forward ***********/
  fillColor = ColorClasses::yellow;

  POLYGON("module:PassHelperProvider:drawPassZone",
          4,
          passZones.Forward_Area,
          0,
          Drawings::ps_null,
          standardColor.penColor,
          Drawings::bs_solid,
          fillColor);

  /*********** Forward / Lateral ***********/
  fillColor = ColorClasses::orange;

  POLYGON("module:PassHelperProvider:drawPassZone",
          4,
          passZones.Forward_Lateral_Area,
          0,
          Drawings::ps_null,
          standardColor.penColor,
          Drawings::bs_solid,
          fillColor);

  /*********** shot ***********/
  fillColor = ColorClasses::red;

  POLYGON("module:PassHelperProvider:drawPassZone",
          4,
          passZones.Shot_Area,
          0,
          Drawings::ps_null,
          standardColor.penColor,
          Drawings::bs_solid,
          fillColor);

  /*********** Backward / Lateral Left ***********/
  fillColor = ColorClasses::green;

  POLYGON("module:PassHelperProvider:drawPassZone",
          4,
          passZones.Lateral_Backward_Left_Area,
          0,
          Drawings::ps_null,
          standardColor.penColor,
          Drawings::bs_solid,
          fillColor);

  /*********** Backward / Lateral Right ***********/
  fillColor = ColorClasses::green;

  POLYGON("module:PassHelperProvider:drawPassZone",
          4,
          passZones.Lateral_Backward_Right_Area,
          0,
          Drawings::ps_null,
          standardColor.penColor,
          Drawings::bs_solid,
          fillColor);

  // Draw the line in order to make the drawing understandable

  // Up End line
  LINE("module:PassHelperProvider:drawPassZone",
       theFieldDimensions.xPosOwnGroundline,
       theFieldDimensions.yPosLeftSideline,
       theFieldDimensions.xPosOpponentGroundline,
       theFieldDimensions.yPosLeftSideline,
       standardColor.penWidth,
       standardColor.penStyle,
       standardColor.penColor);

  // Down End line
  LINE("module:PassHelperProvider:drawPassZone",
       theFieldDimensions.xPosOwnGroundline,
       theFieldDimensions.yPosRightSideline,
       theFieldDimensions.xPosOpponentGroundline,
       theFieldDimensions.yPosRightSideline,
       standardColor.penWidth,
       standardColor.penStyle,
       standardColor.penColor);

  // Left End line
  LINE("module:PassHelperProvider:drawPassZone",
       theFieldDimensions.xPosOwnGroundline,
       theFieldDimensions.yPosLeftSideline,
       theFieldDimensions.xPosOwnGroundline,
       theFieldDimensions.yPosRightSideline,
       standardColor.penWidth,
       standardColor.penStyle,
       standardColor.penColor);

  // Right End line
  LINE("module:PassHelperProvider:drawPassZone",
       theFieldDimensions.xPosOpponentGroundline,
       theFieldDimensions.yPosLeftSideline,
       theFieldDimensions.xPosOpponentGroundline,
       theFieldDimensions.yPosRightSideline,
       standardColor.penWidth,
       standardColor.penStyle,
       standardColor.penColor);

  // Middle line
  LINE("module:PassHelperProvider:drawPassZone",
       0,
       theFieldDimensions.yPosLeftSideline,
       0,
       theFieldDimensions.yPosRightSideline,
       standardColor.penWidth,
       standardColor.penStyle,
       standardColor.penColor);

  // Middle Circle
  CIRCLE("module:PassHelperProvider:drawPassZone",
         0,
         0,
         theFieldDimensions.centerCircleRadius,
         standardColor.penWidth,
         standardColor.penStyle,
         standardColor.penColor,
         standardColor.fillStyle,
         standardColor.fillColor);

  // Our penalty box: Left line
  LINE("module:PassHelperProvider:drawPassZone",
       theFieldDimensions.xPosOwnGroundline,
       theFieldDimensions.yPosLeftPenaltyArea,
       theFieldDimensions.xPosOwnPenaltyArea,
       theFieldDimensions.yPosLeftPenaltyArea,
       standardColor.penWidth,
       standardColor.penStyle,
       standardColor.penColor);

  // Our penalty box: Right line
  LINE("module:PassHelperProvider:drawPassZone",
       theFieldDimensions.xPosOwnGroundline,
       theFieldDimensions.yPosRightPenaltyArea,
       theFieldDimensions.xPosOwnPenaltyArea,
       theFieldDimensions.yPosRightPenaltyArea,
       standardColor.penWidth,
       standardColor.penStyle,
       standardColor.penColor);

  // Our penalty box: y line
  LINE("module:PassHelperProvider:drawPassZone",
       theFieldDimensions.xPosOwnPenaltyArea,
       theFieldDimensions.yPosLeftPenaltyArea,
       theFieldDimensions.xPosOwnPenaltyArea,
       theFieldDimensions.yPosRightPenaltyArea,
       standardColor.penWidth,
       standardColor.penStyle,
       standardColor.penColor);

  // Opponent penalty box: Left line
  LINE("module:PassHelperProvider:drawPassZone",
       theFieldDimensions.xPosOpponentGroundline,
       theFieldDimensions.yPosLeftPenaltyArea,
       theFieldDimensions.xPosOpponentPenaltyArea,
       theFieldDimensions.yPosLeftPenaltyArea,
       standardColor.penWidth,
       standardColor.penStyle,
       standardColor.penColor);

  // Opponent penalty box: Right line
  LINE("module:PassHelperProvider:drawPassZone",
       theFieldDimensions.xPosOpponentGroundline,
       theFieldDimensions.yPosRightPenaltyArea,
       theFieldDimensions.xPosOpponentPenaltyArea,
       theFieldDimensions.yPosRightPenaltyArea,
       standardColor.penWidth,
       standardColor.penStyle,
       standardColor.penColor);

  // Opponent penalty box: y line
  LINE("module:PassHelperProvider:drawPassZone",
       theFieldDimensions.xPosOpponentPenaltyArea,
       theFieldDimensions.yPosLeftPenaltyArea,
       theFieldDimensions.xPosOpponentPenaltyArea,
       theFieldDimensions.yPosRightPenaltyArea,
       standardColor.penWidth,
       standardColor.penStyle,
       standardColor.penColor);

  // Our Penalty Mark
  CROSS("module:PassHelperProvider:drawPassZone",
        theFieldDimensions.xPosOwnPenaltyMark,
        0,
        50,
        100,
        Drawings::ps_solid,
        standardColor.penColor);

  // Opponent Penalty Mark
  CROSS("module:PassHelperProvider:drawPassZone",
        theFieldDimensions.xPosOpponentPenaltyMark,
        0,
        50,
        100,
        Drawings::ps_solid,
        standardColor.penColor);

  // Middle Point
  CIRCLE("module:PassHelperProvider:drawPassZone",
         0,
         0,
         50,
         standardColor.penWidth,
         Drawings::ps_solid,
         ColorClasses::black,
         Drawings::bs_solid,
         standardColor.penColor);

  // Draw the Legend
  legendOptions.startLegend();

  if (drawingOptions.VisualizeLegend) {
    // Draw the Legend
    FILLED_RECTANGLE("module:PassHelperProvider:drawPassZone",
                     legendOptions.sLowerLeftX,
                     legendOptions.textY,
                     legendOptions.sUpperRightX,
                     legendOptions.sUpperRightY,
                     0,
                     Drawings::PenStyle::ps_solid,
                     standardColor.penColor,
                     Drawings::FillStyle::bs_solid,
                     ColorClasses::yellow);

    DRAWTEXT("module:PassHelperProvider:drawPassZone",
             legendOptions.textX,
             legendOptions.textY,
             legendOptions.Font,
             ColorClasses::black,
             "Forward");

    legendOptions.nextLine();

    // Draw the Legend
    FILLED_RECTANGLE("module:PassHelperProvider:drawPassZone",
                     legendOptions.sLowerLeftX,
                     legendOptions.textY,
                     legendOptions.sUpperRightX,
                     legendOptions.sUpperRightY,
                     0,
                     Drawings::PenStyle::ps_solid,
                     standardColor.penColor,
                     Drawings::FillStyle::bs_solid,
                     ColorClasses::orange);

    DRAWTEXT("module:PassHelperProvider:drawPassZone",
             legendOptions.textX,
             legendOptions.textY,
             legendOptions.Font,
             ColorClasses::black,
             "For. / Lat.");

    legendOptions.nextLine();

    // Draw the Legend
    FILLED_RECTANGLE("module:PassHelperProvider:drawPassZone",
                     legendOptions.sLowerLeftX,
                     legendOptions.textY,
                     legendOptions.sUpperRightX,
                     legendOptions.sUpperRightY,
                     0,
                     Drawings::PenStyle::ps_solid,
                     standardColor.penColor,
                     Drawings::FillStyle::bs_solid,
                     ColorClasses::red);

    DRAWTEXT("module:PassHelperProvider:drawPassZone",
             legendOptions.textX,
             legendOptions.textY,
             legendOptions.Font,
             ColorClasses::black,
             "Shoot");

    legendOptions.nextLine();

    // Draw the Legend
    FILLED_RECTANGLE("module:PassHelperProvider:drawPassZone",
                     legendOptions.sLowerLeftX,
                     legendOptions.textY,
                     legendOptions.sUpperRightX,
                     legendOptions.sUpperRightY,
                     0,
                     Drawings::PenStyle::ps_solid,
                     standardColor.penColor,
                     Drawings::FillStyle::bs_solid,
                     ColorClasses::green);

    DRAWTEXT("module:PassHelperProvider:drawPassZone",
             legendOptions.textX,
             legendOptions.textY,
             legendOptions.Font,
             ColorClasses::black,
             "Lat. / Back.");
  }
}

void PassHelperProvider::drawAllRobot() {
  DECLARE_DEBUG_DRAWING("module:PassHelperProvider:drawAllRobot", "drawingOnField");

  // First calculate the shadow for our team
  for (int i = 1; i < theTeamMateData.numOfPlayers; i++) {
    // If the robot from which we want to calculate the shadow is the robot itself, just skip
    // No need to make calculation for the robot which will pass the ball
    if (i == theRobotInfo.number) {
      continue;
    }

    // If a robot is not active, then we don't need to calculate the shadow
    if (!theTeamMateData.isActive[i]) {
      continue;
    }

    DRAWTEXT("module:PassHelperProvider:drawPossibleTargetPoints",
             theTeamMateData.robotPoses[i].translation.x,
             theTeamMateData.robotPoses[i].translation.y,
             30,
             ColorClasses::red,
             i);

    // DRAW_ROBOT_POSE("module:PassHelperProvider:drawAllRobot", theTeamMateData.robotPoses[i], ColorClasses::black);
    // Unfortunately this function DRAW_ROBOT_POSE doesn't work as supposed..
  }
}

void PassHelperProvider::decidePassPoint() {
  // First reset our variable: by default used the center of the opponents door
  foundPassPoint = false;
  typePass = PassHelper::shot;
  distancePass = 10000; // Use the maximum strength
  idRecieverPass = 0;
  // This point represent the center of the opponent goal post
  passCoordinate = Vector2<>(theFieldDimensions.xPosOpponentGoalPost, 0);

  // First check in which area is the ball and then according to that choose the kind of pass that we want to perform
  if (theCombinedWorldModel.ballState.position.x <=
      passZones.Forward_Area[1].x) // = theFieldDimensions.xPosHalfWayLine = 0 (checked)
  {
    // Yellow area
    decidePassPointForward();
  } else {
    if (theCombinedWorldModel.ballState.position.x <= passZones.Forward_Lateral_Area[1].x) {
      // Orange area
      decidePassPointForward();
      if (!foundPassPoint) {
        decidePassPointLateral();
      }
    } else // We have theCombinedWorldModel.ballState.position.x > theFieldDimensions.xPosOpponentPenaltyMark
    {
      // Note: left = y > 0 ; right = y < 0

      StraightLine<> straightLeft(passZones.Lateral_Backward_Left_Area[2], passZones.Lateral_Backward_Left_Area[3]);
      StraightLine<> straightRight(passZones.Lateral_Backward_Right_Area[0], passZones.Lateral_Backward_Right_Area[1]);

      float yLeft, yRight, x, y;
      x = theCombinedWorldModel.ballState.position.x;
      y = theCombinedWorldModel.ballState.position.y;

      yLeft = straightLeft.m * x + straightLeft.q;
      yRight = straightRight.m * x + straightRight.q;

      if ((y < yLeft) && (y > yRight)) {
        // Red area
        foundPassPoint = false;
      } else {
        // Green
        decidePassPointLateral();
        if (!foundPassPoint) {
          decidePassPointBackward();
        }
      }
    }
  }

  // Now we know, that if pointFound == true, that mean that we found a point where to pass the case. If point found ==
  // false, this means that we couldn't find
  // a point where shoot the ball, and therefore we will wither shot to the goal or dribbling

  // Calculate the distance between the ball and the pass point. This in order to calculate the strength of the shoot
  if (foundPassPoint) {
    distancePass = std::sqrt(std::pow(theCombinedWorldModel.ballState.position.x - passCoordinate.x, 2) +
                             std::pow(theCombinedWorldModel.ballState.position.y - passCoordinate.y, 2));
  }
}

void PassHelperProvider::decidePassPointForward() {
  // See documentation for more detail

  // Consider in which band we are
  bool centralBandBall;
  bool centralBandPoint;

  if ((-parameters.halfBandDistanceForwardPass < theCombinedWorldModel.ballState.position.y) &&
      (theCombinedWorldModel.ballState.position.y < parameters.halfBandDistanceForwardPass)) {
    centralBandBall = true;
  } else {
    centralBandBall = false;
  }

  int yPoint;

  // search for every player
  for (int i = 0; i < theTeamMateData.numOfPlayers; i++) {
    // Iterate through all the possible points
    for (std::vector<Index>::iterator it = selectedPointsIndex[i].begin(); it != selectedPointsIndex[i].end(); ++it) {
      // Since this is a forward pass, consider just the point with a x > x_position_passing_robot
      if (pointsPtr[it->first][it->second].first.x < theCombinedWorldModel.ballState.position.x) {
        continue;
      }

      // check if the points is in our band and in that case discard it
      yPoint = pointsPtr[it->first][it->second].first.y;

      // First look in which band is the ball
      if ((-parameters.halfBandDistanceForwardPass < yPoint) && (yPoint < parameters.halfBandDistanceForwardPass)) {
        centralBandPoint = true;
      } else {
        centralBandPoint = false;
      }

      // If both are in the same band, don't consider the point
      if (centralBandBall == centralBandPoint) {
        continue;
      }

      // Check if the point is in a reachable distance from the passing robot
      if (insidePassFrontRange(*it)) {
        if (foundPassPoint) {
          // In the case that we already found a point, choose the one which is more further
          if (passCoordinate.x < pointsPtr[it->first][it->second].first.x) {
            passCoordinate = pointsPtr[it->first][it->second].first;
            idRecieverPass = i;
          }

          // If the point has the same x, choose the one with the further y
          if (passCoordinate.x == pointsPtr[it->first][it->second].first.x) {
            int dyNew = std::abs(pointsPtr[it->first][it->second].first.y - theCombinedWorldModel.ballState.position.y);
            int dyConsidered = std::abs(passCoordinate.y - theCombinedWorldModel.ballState.position.y);

            if (dyConsidered < dyNew) {
              passCoordinate = pointsPtr[it->first][it->second].first;
              idRecieverPass = i;
            }
          }
        } else {
          passCoordinate = pointsPtr[it->first][it->second].first;
          foundPassPoint = true;
          idRecieverPass = i;
        }
      }
    }
  }
  if (foundPassPoint) {
    typePass = PassHelper::forward;
  }
}

void PassHelperProvider::decidePassPointBackward() {
  // Consider in which band we are
  bool centralBandBall;
  bool centralBandPoint;

  // Decide in which band is the robot
  if ((-parameters.halfBandDistanceForwardPass < theCombinedWorldModel.ballState.position.y) &&
      (theCombinedWorldModel.ballState.position.y < parameters.halfBandDistanceForwardPass)) {
    centralBandBall = true;
  } else {
    centralBandBall = false;
  }

  int yPoint;

  // search for every player
  for (int i = 0; i < theTeamMateData.numOfPlayers; i++) {
    // Iterate through all the possible points
    for (std::vector<Index>::iterator it = selectedPointsIndex[i].begin(); it != selectedPointsIndex[i].end(); ++it) {
      // Since this is a backward pass, consider just the point with a x < x_position_passing_robot
      if (theCombinedWorldModel.ballState.position.x < pointsPtr[it->first][it->second].first.x) {
        continue;
      }

      // check if the points is in our band and in that case discard it
      yPoint = pointsPtr[it->first][it->second].first.y;

      // First look in which band is the ball
      if ((-parameters.halfBandDistanceForwardPass < yPoint) && (yPoint < parameters.halfBandDistanceForwardPass)) {
        centralBandPoint = true;
      } else {
        centralBandPoint = false;
      }

      // If both are in the same band, don't consider the point
      if (centralBandBall == centralBandPoint) {
        continue;
      }

      // Check if the point is in a reachable distance from the passing robot
      if (insidePassFrontRange(*it)) {
        if (foundPassPoint) {
          // In the case that we already found a point, choose the one which is more further
          if (pointsPtr[it->first][it->second].first.x < passCoordinate.x) {
            passCoordinate = pointsPtr[it->first][it->second].first;
            idRecieverPass = i;
          }

          // If the point has the same x, choose the one with the further y
          if (passCoordinate.x == pointsPtr[it->first][it->second].first.x) {
            int dyNew = std::abs(pointsPtr[it->first][it->second].first.y - theCombinedWorldModel.ballState.position.y);
            int dyConsidered = std::abs(passCoordinate.y - theCombinedWorldModel.ballState.position.y);

            if (dyConsidered < dyNew) {
              passCoordinate = pointsPtr[it->first][it->second].first;
              idRecieverPass = i;
            }
          }
        }

        else {
          passCoordinate = pointsPtr[it->first][it->second].first;
          foundPassPoint = true;
          idRecieverPass = i;
        }
      }
    }
  }
  if (foundPassPoint) {
    typePass = PassHelper::backward;
  }
}

void PassHelperProvider::decidePassPointLateral() {
  float dxNew, dxConsidered;
  float dyNew, dyConsidered;

  // search for every player
  for (int i = 0; i < theTeamMateData.numOfPlayers; i++) {
    // Iterate through all the possible points
    for (std::vector<Index>::iterator it = selectedPointsIndex[i].begin(); it != selectedPointsIndex[i].end(); ++it) {
      // Check if the point is in a reachable distance from the passing robot
      if (insidePassLateralRange(*it)) {
        if (foundPassPoint) {
          dxNew = std::abs(pointsPtr[it->first][it->second].first.x - theCombinedWorldModel.ballState.position.x);
          dxConsidered = std::abs(passCoordinate.x - theCombinedWorldModel.ballState.position.x);

          // In the case that we already found a point, choose the one which is more closest to the x of the ball
          if (dxNew < dxConsidered) {
            passCoordinate = pointsPtr[it->first][it->second].first;
            idRecieverPass = i;
          }

          // In the case that we have the same x, choose the one with the y smaller
          if (dxNew == dxConsidered) {
            dyNew = std::abs(pointsPtr[it->first][it->second].first.y - theCombinedWorldModel.ballState.position.y);
            dyConsidered = std::abs(passCoordinate.y - theCombinedWorldModel.ballState.position.y);

            if (dyNew < dyConsidered) {
              passCoordinate = pointsPtr[it->first][it->second].first;
              idRecieverPass = i;
            }
          }
        } else {
          passCoordinate = pointsPtr[it->first][it->second].first;
          foundPassPoint = true;
          idRecieverPass = i;
        }
      }
    }
  }
  if (foundPassPoint) {
    typePass = PassHelper::lateral;
  }
}

bool PassHelperProvider::insidePassFrontRange(Index Point) {
  float dist = ballDistanceToPoint(Point);

  if ((dist < parameters.minDistFrontKick) || (parameters.maxDistFrontKick < dist)) {
    return false;
  } else {
    return true;
  }
}

float PassHelperProvider::robotDistanceToPoint(Index Point) {
  return (theRobotPose.translation - pointsPtr[Point.first][Point.second].first).abs();
}

float PassHelperProvider::ballDistanceToPoint(Index Point) {
  return (theCombinedWorldModel.ballState.position - pointsPtr[Point.first][Point.second].first).abs();
}

bool PassHelperProvider::insidePassLateralRange(Index Point) {
  // Since this is a lateral pass, consider just the point with a x included in some limit band
  if (parameters.halfBandXDistanceLateralKick <
      std::abs(theCombinedWorldModel.ballState.position.x - pointsPtr[Point.first][Point.second].first.x)) {
    return false;
  }

  // check if the points is in our y band and if not that case discard it
  int yPoint = pointsPtr[Point.first][Point.second].first.y;

  // check if the pass point is inside the upper or the lower bounding box
  if (((theCombinedWorldModel.ballState.position.y + parameters.minDistYLateralKick) < yPoint &&
       yPoint < (theCombinedWorldModel.ballState.position.y + parameters.maxDistYLateralKick)) ||
      ((theCombinedWorldModel.ballState.position.y - parameters.maxDistYLateralKick) < yPoint &&
       yPoint < (theCombinedWorldModel.ballState.position.y - parameters.minDistYLateralKick))) {
    return true;
  } else {
    return false;
  }
}

void PassHelperProvider::drawPassPoint() {
  DECLARE_DEBUG_DRAWING("module:PassHelperProvider:drawPassPoint", "drawingOnField");

  //*************** Draw the BOUNDARY given by the FORWARD pass

  // yUp
  LINE("module:PassHelperProvider:drawPassPoint",
       theFieldDimensions.xPosOwnGroundline,
       parameters.halfBandDistanceForwardPass,
       theFieldDimensions.xPosOpponentGroundline,
       parameters.halfBandDistanceForwardPass,
       30,
       standardColor.penStyle,
       ColorClasses::orange);
  // yDown
  LINE("module:PassHelperProvider:drawPassPoint",
       theFieldDimensions.xPosOwnGroundline,
       -parameters.halfBandDistanceForwardPass,
       theFieldDimensions.xPosOpponentGroundline,
       -parameters.halfBandDistanceForwardPass,
       30,
       standardColor.penStyle,
       ColorClasses::orange);

  //*************** Draw two circle around the Passing player to represent MINIMUM and MAXIMUM or the FRONT PASS
  // Min
  CIRCLE("module:PassHelperProvider:drawPassPoint",
         theCombinedWorldModel.ballState.position.x,
         theCombinedWorldModel.ballState.position.y,
         parameters.minDistFrontKick,
         standardColor.penWidth,
         standardColor.penStyle,
         standardColor.penColor,
         standardColor.fillStyle,
         standardColor.fillColor);

  // Max
  CIRCLE("module:PassHelperProvider:drawPassPoint",
         theCombinedWorldModel.ballState.position.x,
         theCombinedWorldModel.ballState.position.y,
         parameters.maxDistFrontKick,
         standardColor.penWidth,
         standardColor.penStyle,
         standardColor.penColor,
         standardColor.fillStyle,
         standardColor.fillColor);

  //*************** Draw 4 bounding vertical line for the SIDE PASS
  // <, > are given considering the ball as origin

  // x > 0, y > 0
  LINE("module:PassHelperProvider:drawPassPoint",
       theCombinedWorldModel.ballState.position.x + parameters.halfBandXDistanceLateralKick,
       theCombinedWorldModel.ballState.position.y + parameters.minDistYLateralKick,
       theCombinedWorldModel.ballState.position.x + parameters.halfBandXDistanceLateralKick,
       theCombinedWorldModel.ballState.position.y + parameters.maxDistYLateralKick,
       30,
       standardColor.penStyle,
       ColorClasses::green);

  // x < 0, y > 0
  LINE("module:PassHelperProvider:drawPassPoint",
       theCombinedWorldModel.ballState.position.x - parameters.halfBandXDistanceLateralKick,
       theCombinedWorldModel.ballState.position.y + parameters.minDistYLateralKick,
       theCombinedWorldModel.ballState.position.x - parameters.halfBandXDistanceLateralKick,
       theCombinedWorldModel.ballState.position.y + parameters.maxDistYLateralKick,
       30,
       standardColor.penStyle,
       ColorClasses::green);

  // x > 0, y < 0
  LINE("module:PassHelperProvider:drawPassPoint",
       theCombinedWorldModel.ballState.position.x + parameters.halfBandXDistanceLateralKick,
       theCombinedWorldModel.ballState.position.y - parameters.minDistYLateralKick,
       theCombinedWorldModel.ballState.position.x + parameters.halfBandXDistanceLateralKick,
       theCombinedWorldModel.ballState.position.y - parameters.maxDistYLateralKick,
       30,
       standardColor.penStyle,
       ColorClasses::green);

  // x < 0, y < 0
  LINE("module:PassHelperProvider:drawPassPoint",
       theCombinedWorldModel.ballState.position.x - parameters.halfBandXDistanceLateralKick,
       theCombinedWorldModel.ballState.position.y - parameters.minDistYLateralKick,
       theCombinedWorldModel.ballState.position.x - parameters.halfBandXDistanceLateralKick,
       theCombinedWorldModel.ballState.position.y - parameters.maxDistYLateralKick,
       30,
       standardColor.penStyle,
       ColorClasses::green);

  //*************** Draw horizontal line for the SIDE PASS

  // y = yBig
  LINE("module:PassHelperProvider:drawPassPoint",
       theCombinedWorldModel.ballState.position.x - parameters.halfBandXDistanceLateralKick,
       theCombinedWorldModel.ballState.position.y + parameters.maxDistYLateralKick,
       theCombinedWorldModel.ballState.position.x + parameters.halfBandXDistanceLateralKick,
       theCombinedWorldModel.ballState.position.y + parameters.maxDistYLateralKick,
       30,
       standardColor.penStyle,
       ColorClasses::green);

  // y = ySmall
  LINE("module:PassHelperProvider:drawPassPoint",
       theCombinedWorldModel.ballState.position.x - parameters.halfBandXDistanceLateralKick,
       theCombinedWorldModel.ballState.position.y + parameters.minDistYLateralKick,
       theCombinedWorldModel.ballState.position.x + parameters.halfBandXDistanceLateralKick,
       theCombinedWorldModel.ballState.position.y + parameters.minDistYLateralKick,
       30,
       standardColor.penStyle,
       ColorClasses::green);

  // y = -ySmall
  LINE("module:PassHelperProvider:drawPassPoint",
       theCombinedWorldModel.ballState.position.x - parameters.halfBandXDistanceLateralKick,
       theCombinedWorldModel.ballState.position.y - parameters.minDistYLateralKick,
       theCombinedWorldModel.ballState.position.x + parameters.halfBandXDistanceLateralKick,
       theCombinedWorldModel.ballState.position.y - parameters.minDistYLateralKick,
       30,
       standardColor.penStyle,
       ColorClasses::green);

  // y = -yBig
  LINE("module:PassHelperProvider:drawPassPoint",
       theCombinedWorldModel.ballState.position.x - parameters.halfBandXDistanceLateralKick,
       theCombinedWorldModel.ballState.position.y - parameters.maxDistYLateralKick,
       theCombinedWorldModel.ballState.position.x + parameters.halfBandXDistanceLateralKick,
       theCombinedWorldModel.ballState.position.y - parameters.maxDistYLateralKick,
       30,
       standardColor.penStyle,
       ColorClasses::green);

  if (foundPassPoint) {
    LARGE_DOT("module:PassHelperProvider:drawPassPoint",
              passCoordinate.x,
              passCoordinate.y,
              standardColor.penColor,
              ColorClasses::green);
  }
}