/**
 * @file PointsField.h
 *
 * PointsField is a class that care all the basic calculation to create all the considered points on the field
 * It also perform some index calculation.
 * Note: Further calculation are done in PassHelperProvider.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Debugging/DebugDrawings.h"
#include "PassHelperProviderParameters.h"

#include "PassTypeDef.h"

class PointsField {
public:
  // Even is not a very good Object Oriented way to code, I decide to make all the attribute of this class public in order to
  // not generate a lot of getters and setters.
  // This also to make the code that uses this class more readable.

  // All the user tuned parameters
  PassHelperProviderParameters parameters;

  int numberPoints; // Total number of points
  int n_row;        // Total number of rows
  int n_column;     // Total number of columns
  float xPosOpponentGroundline;
  float yPosLeftSideline;

  // Constructor
  PointsField(float a_xPosOpponentGroundline, float a_yPosLeftSideline);

  // Decide how many points will be used, according to the used defined parameters distancePoints
  void decideNumberOfPoints();

  // Give a double pointers (i.e. able to point to a "matrix' structure), create all the points to which the pointer will
  // point and accessible using pointsPtr[row][column]
  void createPoints(LocTarget**& pointsPtr);

  // Initialize every point, giving to it is location in (x,y) and the default value true (= valid point to pass the ball)
  void populate(LocTarget** pointsPtr);

  // Draw all the points on the field
  void drawAllPoints(LocTarget** pointsPtr);

  // Find the index of the robot (row, column), given his position in (x,y)
  Index findIndexRobot(Location);
};
