/**
 * @file ShootOut.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include <cstdlib>
#include <cmath>

bool PickCornerForPenalty() {
  std::srand(std::time(NULL));
  int pick = rand() % 100;
  if (pick < 50) {
    return true;
  } else {
    return false;
  }
}

float CalculateKickAngleOffset() {
  return std::atan(0.75f * theFieldDimensions.yPosLeftGoal /
                   std::abs(theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyMark));
}