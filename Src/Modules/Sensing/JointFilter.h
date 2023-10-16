/**
 * @file JointFilter.h
 *
 * Declaration of module JointFilter.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Infrastructure/JointData.h"

MODULE(JointFilter)
REQUIRES(JointData)
PROVIDES_WITH_MODIFY_AND_OUTPUT(FilteredJointData)
END_MODULE

/**
 * @class JointFilter
 * A module for sensor data filtering.
 */
class JointFilter : public JointFilterBase {
  /**
   * Updates the FilteredJointData representation .
   * @param filteredJointData The joint data representation which is updated by this module.
   */
  void update(FilteredJointData& filteredJointData);
};
