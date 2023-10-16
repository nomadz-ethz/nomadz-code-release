/**
 * @file PlayerLocator.h
 *
 * Declares a class that estimates the position of the players
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Perception/PlayerPercept.h"
#include "Representations/Modeling/PlayerModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"

MODULE(PlayerLocator)
REQUIRES(PlayerPercept)
REQUIRES(FrameInfo)
REQUIRES(RobotPose)
REQUIRES(Odometer)
REQUIRES(FieldDimensions)
PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(PlayerModel)
LOADS_PARAMETER(float, perceptOffset)            // offset in the x direction to add to percepted players
LOADS_PARAMETER(float, mergeThreshold)           // threshold to merge players
LOADS_PARAMETER(float, initTime)                 // time before percepted player is accepted
LOADS_PARAMETER(float, minVariance)              // minimum required variance a perception should achieve to be accepted
LOADS_PARAMETER(float, rememberingTime)          // maximum time to remember a player
LOADS_PARAMETER(bool, outsideFieldCheck)         // use check for detections outside field
LOADS_PARAMETER(float, playerOutsideFieldOffset) // offset to consider inside field
LOADS_PARAMETER(float, distanceThreshold)        // maximum distance to accept as reliable player percept
LOADS_PARAMETER(bool, enableLogging) /**< generates additional and customizable logfiles, which can be read in Matlab>*/
END_MODULE

/**
 * @class PlayerLocator
 */
class PlayerLocator : public PlayerLocatorBase {
public:
  /**
   * Default constructor.
   */
  PlayerLocator();

  bool isPlayerOutsideOfField(const PlayerModel::Player& player);

private:
  bool init;

  /**
   * Provides player model representation
   */
  void update(PlayerModel& playerModel);
};
