/**
 * @file GlobalFieldCoverage.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Felix Wenk
 */

#include <cstring>
#include <algorithm>
#include "GlobalFieldCoverage.h"
#include "Core/System/BHAssert.h"
#include "Core/Math/Common.h"

unsigned char GlobalFieldCoverage::Grid::coverage(int index, unsigned time) const {
  int sub = (time - cells[index]) / FieldCoverage::GridInterval::tick;
  if (sub < 0) {
    sub = 0;
  }
  return sub >= FieldCoverage::GridInterval::maxCoverage ? 0
                                                         : (unsigned char)(FieldCoverage::GridInterval::maxCoverage - sub);
}

void GlobalFieldCoverage::Grid::setCoverage(int index, unsigned time, unsigned char coverage) {
  ASSERT(coverage <= FieldCoverage::GridInterval::maxCoverage);
  ASSERT(index < FieldCoverage::GridInterval::xSteps * FieldCoverage::GridInterval::ySteps);
  cells[index] = std::max(
    0, static_cast<int>(time - FieldCoverage::GridInterval::tick * (FieldCoverage::GridInterval::maxCoverage - coverage)));
}
