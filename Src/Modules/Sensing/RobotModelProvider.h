/**
 * @file RobotModelProvider.h
 *
 * This file declares a module that provides information about the current state of the robot's limbs.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Infrastructure/JointData.h"

MODULE(RobotModelProvider)
REQUIRES(RobotDimensions)
REQUIRES(FilteredJointData)
REQUIRES(MassCalibration)
PROVIDES_WITH_MODIFY_AND_DRAW(RobotModel)
END_MODULE

/**
 * @class RobotModelProvider
 *
 * A module for computing the current state of the robot's limbs
 */
class RobotModelProvider : public RobotModelProviderBase {
private:
  /** Executes this module
   * @param robotModel The data structure that is filled by this module
   */
  void update(RobotModel& robotModel);
};
