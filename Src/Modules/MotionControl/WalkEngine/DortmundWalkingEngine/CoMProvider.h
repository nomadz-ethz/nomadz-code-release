/**
 * @file CoMProvider.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once
#include <list>
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/DortmundWalkingEngine/StepData.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/MotionControl/ActualCoM.h"

/**
 * @class CoMProvider
 * Determines the target orientation of the body.
 */

class CoMProvider {
public:
  /** Constructor with all needed source data structures.
   * @param theSensorData Measured data.
   * @param theWalkingEngineParams Walking Engine Parameters.
   */
  CoMProvider(
    // const JointAngles				&theJointAngles,
    // const WalkingEngineParams	&theWalkingEngineParams,
    // const JointRequest			&theJointRequest,
    const WalkingInfo& theWalkingInfo,
    // const FootSteps				&theFootSteps,
    // const RobotModel &theRobotModel,
    const ActualCoMRCS& theActualCoMProvider);

  /** Destructor */
  ~CoMProvider(void);

  /**
   * Calculates the next target orientation.
   * @param bodyTilt Target data structure.
   */
  void updateActualCoM(ActualCoM& theActualCoM);

private:
  // const JointAngles				&theJointAngles; unused
  // const WalkingEngineParams	&theWalkingEngineParams;
  // const JointRequest			&theJointRequest;			/**< Set by constructor. */ unused
  const WalkingInfo& theWalkingInfo;
  // const FootSteps				&theFootSteps;

  // Rensen: Removed due to unused warning
  // const RobotModel &theRobotModel;

  const ActualCoMRCS& theActualCoMRCS;

  typedef std::list<Footposition> FootList;

  /** List of target foot positions in world coordinate system filled with the
    foot positions found in theFootpositions. No more needed positions
    are deleted in every step cycle. */
  FootList footPositions;
};
