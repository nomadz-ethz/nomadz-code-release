/**
 * @file LegendOptions.h
 *
 * Class used for the creation of legend in the drawings
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once
#include "Core/Streams/AutoStreamable.h"

STREAMABLE(LegendOptions, {
public:
  LegendOptions(int cornerX, int cornerY);
  void nextLine();
  void startLegend(), (int)startX, (int)startY, (int)sLowerLeftX, (int)sLowerLeftY, (int)sUpperRightX, (int)sUpperRightY,
    (int)textX, (int)textY, (int)dimSquare, (int)verticalDistance, (int)distanceSquareText, (int)Font,
});

LegendOptions::LegendOptions(int cornerX, int cornerY) {
  //================================ Parameter to modify according to the user need
  Font = 40;
  dimSquare = 400;
  verticalDistance = 1000;
  distanceSquareText = 200;
  startX = cornerX + 200;
  startY = cornerY - 700;
  //================================

  sLowerLeftX = startX;
  sLowerLeftY = startY;
  sUpperRightX = sLowerLeftX + dimSquare;
  sUpperRightY = sLowerLeftY + dimSquare;

  textX = sUpperRightX + distanceSquareText;
  textY = sLowerLeftY;
}

void LegendOptions::startLegend() {
  sLowerLeftX = startX;
  sLowerLeftY = startY;
  sUpperRightX = sLowerLeftX + dimSquare;
  sUpperRightY = sLowerLeftY + dimSquare;

  textX = sUpperRightX + distanceSquareText;
  textY = sLowerLeftY;
}

void LegendOptions::nextLine() {
  sLowerLeftY -= verticalDistance;
  sUpperRightY -= verticalDistance;

  textX = sUpperRightX + distanceSquareText;
  textY = sLowerLeftY;
}