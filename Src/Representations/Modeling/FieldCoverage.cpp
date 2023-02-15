/**
 * @file FieldCoverage.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Felix Wenk
 */

#include "Core/System/BHAssert.h"
#include "FieldCoverage.h"

FieldCoverage::Target::Target(float x, float y, unsigned short coverage, bool valid) : Target() {
  target.x = x;
  target.y = y;
  this->coverage = coverage;
  isValid = valid;
}

FieldCoverage::Target::Target(const Vector2<>& target, unsigned short coverage, bool valid) : Target() {
  this->target = target;
  this->coverage = coverage;
  isValid = valid;
}

unsigned char FieldCoverage::coverage(int cellIdx, unsigned time) const {
  int sub = (time - cells[cellIdx]) / GridInterval::tick;
  ASSERT(sub >= 0);
  return sub >= GridInterval::maxCoverage ? 0 : static_cast<unsigned char>(GridInterval::maxCoverage - sub);
}
