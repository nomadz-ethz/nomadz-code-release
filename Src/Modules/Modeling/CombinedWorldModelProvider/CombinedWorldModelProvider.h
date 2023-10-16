/**
 * @file CombinedWorldModelProvider.h
 *
 * Declares a class that provides a combined world model
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Katharina Gillmann
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/PlayerModel.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Core/RingBuffer.h"
#include "Core/Math/Vector.h"
#include "Core/Math/Matrix.h"
#include "Ally.h"
#include "TrackedRobot.h"
#include "ExtendedBallModel.h"

MODULE(CombinedWorldModelProvider)
REQUIRES(RobotPose)
REQUIRES(RobotInfo)
REQUIRES(PlayerModel)
REQUIRES(BallModel)
REQUIRES(TeamMateData)
REQUIRES(OwnTeamInfo)
REQUIRES(FieldDimensions)
REQUIRES(FrameInfo)
REQUIRES(FallDownState)
REQUIRES(GroundContactState)
REQUIRES(CameraMatrix)
USES(CombinedWorldModel)
USES(SideConfidence)
PROVIDES_WITH_MODIFY_AND_DRAW(CombinedWorldModel)
LOADS_PARAMETER(
  float,
  movementFactorBallDisappeared) /**< factor for the movement of the sigmoid function for the ball disappeared weight */
LOADS_PARAMETER(float, movementFactorBallSinceLastSeen) /**< factor for the movement of the sigmoid function for the ball
                                                           time since last seen weight */
LOADS_PARAMETER(
  float, scalingFactorBallDisappeared) /**< factor for the scaling of the sigmoid function for the ball disappeared weight */
LOADS_PARAMETER(float, scalingFactorBallSinceLastSeen) /**< factor for the scaling of the sigmoid function for the ball time
                                                          since last seen weight */
LOADS_PARAMETER(float, clusteringDistance) /**< The distance between obstacles which are added to the same cluster */
LOADS_PARAMETER(float, distanceToTeamMate) /**< distance of an obstacle to an own teammate */
LOADS_PARAMETER(int, ballModelAge)         /**< minimum age of the old used ballModel>*/
LOADS_PARAMETER(int,
                ballModelOthersTimeOut) /**< maximum age of a ball model that can be integrated into the ballStateOthers */
LOADS_PARAMETER(bool, closeRobotsNeedLocalDetection) /**< activates additional constraint to avoid close false positives
                                                        resulting from self-localization errors of teammates */
LOADS_PARAMETER(float, closeRobotDetectionDistance)  /**< distance up to which a local detection is require, if
                                                        closeRobotsNeedLocalDetection is true */
LOADS_PARAMETER(float, playerDistanceThreshold)      /**< used to fuse different playerModel information together */
LOADS_PARAMETER(double, validityThreshold) /**< the validity value measures the confidence in the selflocalisation result */
LOADS_PARAMETER(bool, enableLogging) /**< generates additional and customizable logfiles, which can be read in Matlab>*/
END_MODULE

/**
 * @class CombinedWorldModelProvider
 * A combined world model
 */
class CombinedWorldModelProvider : public CombinedWorldModelProviderBase {
public:
  CombinedWorldModelProvider() {}

  /**
   * Provides the combined world model representation
   */
  void update(CombinedWorldModel& combinedWorldModel);

  /**
   * Provides the combined ball model representation by teammates only
   */
  void updateOthers(CombinedWorldModel& combinedWorldModel);

  /*-------------------- Robot Positions update--------------------- */
  // design parameters
  const float initDistance = 600;    // [mm] detections only create a track if they are further away from any other tracks
  const int stopTrackTime = 8000;    //[ms] time after which a undetected track is terminated
  const int trackCreationTime = 200; //[ms] time after which a new track is created
  const float playerOutsideFieldOffset =
    500; //[mm] offset from field lines where tracking stops !! This function is implemented at multiple different places!!
  const float maxTotalVar = 1000; // measurement for the total uncertainty of the tracked robots
  std::vector<unsigned> arrivalTimes = std::vector<unsigned>(TeamMateData::numOfPlayers);
  unsigned latestUpdate = 0;
  std::vector<Ally> allies = std::vector<Ally>(TeamMateData::numOfPlayers);
  std::vector<TrackedRobot> opponents;
  std::vector<TrackedRobot> potentialOpponents;
  bool isPlayerOutsideOfField(TrackedRobot& robot);

  /*---------------------------- Ball Model Update ---------------------*/
  std::vector<ExtendedBallModel> potentialBallModels;
  ExtendedBallModel selectedBallModel;
  ExtendedBallModel selectedBallModelOthers;
  const float ballDistanceThreshold = 500;
  const float minSeenFraction = 0.1;
  const int deleteBallTime = 5000; //[ms] time after which a Bal Model is removed from the tracked list
};
