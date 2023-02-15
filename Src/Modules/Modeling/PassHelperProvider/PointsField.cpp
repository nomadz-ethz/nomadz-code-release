/**
 * @file PointsField.cpp
 *
 * Implementation of PointsField.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "PointsField.h"

PointsField::PointsField(float a_xPosOpponentGroundline, float a_yPosLeftSideline) {
  xPosOpponentGroundline = a_xPosOpponentGroundline;
  yPosLeftSideline = a_yPosLeftSideline;

  numberPoints = 0;
  n_row = 0;
  n_column = 0;
}

void PointsField::decideNumberOfPoints() {
  int dimX, dimY, pointX, pointY;

  // Find how much point for direction we want
  dimX = 2 * xPosOpponentGroundline;
  dimY = 2 * yPosLeftSideline;

  // Calculate the number of point that we will use
  pointX = floor(dimX / parameters.distancePoints);
  pointY = floor(dimY / parameters.distancePoints);

  // Check if we have even number: in that case remove 1 (since mirrored from the y axis)
  if (pointX % 2 == 0) {
    pointX = pointX - 1;
  }
  if (pointY % 2 == 0) {
    pointY = pointY - 1;
  }

  n_column = pointX;
  n_row = pointY;
  numberPoints = pointX * pointY;
}

void PointsField::createPoints(LocTarget**& pointsPtr) {
  pointsPtr = new LocTarget*[n_row];
  for (int i = 0; i < n_row; i++) {
    pointsPtr[i] = new LocTarget[n_column];
  }
}

void PointsField::populate(LocTarget** pointsPtr) {
  // Note: Access the structure:

  //[0][0] ------------> [0][m_column -1]
  //   :						  :
  //   :						  :
  //[m_row-1][0] ------>  [m_row-1][m_column -1]

  int idC = floor(n_column / 2);
  int idR = floor(n_row / 2);

  for (int row = 0; row < n_row; row++) {
    for (int column = 0; column < n_column; column++) {
      pointsPtr[row][column].first =
        Vector2<>((column - idC) * parameters.distancePoints, (idR - row) * parameters.distancePoints);
      pointsPtr[row][column].second = true;
    }
  }
}

void PointsField::drawAllPoints(LocTarget** pointsPtr) {
  ColorRGBA color = ColorClasses::black;
  DECLARE_DEBUG_DRAWING("module:PassHelperProvider:drawAllPoints", "drawingOnField");

  for (int row = 0; row < n_row; row++) {
    for (int column = 0; column < n_column; column++) {
      LARGE_DOT("module:PassHelperProvider:drawAllPoints",
                pointsPtr[row][column].first.x,
                pointsPtr[row][column].first.y,
                color,
                color);
    }
  }
}

Index PointsField::findIndexRobot(Location robotLocation) {
  int column, row;

  column = (parameters.distancePoints * floor(n_column / 2) + robotLocation.x + parameters.distancePoints / 2) /
           parameters.distancePoints;
  row = (parameters.distancePoints * floor(n_row / 2) - robotLocation.y + parameters.distancePoints / 2) /
        parameters.distancePoints;

  if (row < 0 || n_row <= row || column < 0 || n_column <= column) {
    return std::make_pair(-1, -1); // i.e. the robot is not on the field
  }

  return std::make_pair(row, column);
}
