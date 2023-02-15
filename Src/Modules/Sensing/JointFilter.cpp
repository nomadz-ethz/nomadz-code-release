/**
 * @file JointFilter.cpp
 *
 * Implementation of module JointFilter.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#include "JointFilter.h"

MAKE_MODULE(JointFilter, Sensing)

void JointFilter::update(FilteredJointData& filteredJointData) {
  for (int i = 0; i < JointData::numOfJoints; ++i) {
    if (theJointData.angles[i] != JointData::off) {
      filteredJointData.angles[i] = theJointData.angles[i];
    } else if (filteredJointData.angles[i] == JointData::off) {
      filteredJointData.angles[i] = 0;
    }
  }
  filteredJointData.timeStamp = theJointData.timeStamp;
}
