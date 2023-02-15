/**
 * @file MotionCombinator.h
 *
 * This file implements a module that combines the motions created by the different modules.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A> and Jesse Richter-Klug
 */

#pragma once

#include "Representations/Configuration/OdometryCorrectionTable.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/OrientationData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Core/Module/Module.h"

STREAMABLE(BalanceParameters,
           {
             ,
             (Angle)targetAngle,
             (float)gyroX_p,
             (float)gyroX_d,
             (float)gyroY_p,
             (float)gyroY_d,
             (float)angleX_p,
             (float)angleX_i,
             (float)angleY_p,
             (float)angleY_i,
             (float)comX_p,
             (float)angleGyroRatioX,
             (float)angleGyroRatioY,
             (float)hipRollFactor,
             (float)hipPitchFactor,
             (float)kneeFactor,
             (float)footPitchFactor,
             (float)footRollFactor,
           });

STREAMABLE(SpecialActionBalanceList, {
  public : STREAMABLE(SpecialActionBalanceEntry,
                      {
                        ,
                        (SpecialActionRequest, SpecialActionID)specialAction,
                        (int)(1000)balanceStartTime,
                        (int)(4000)specialActionDuration,
                        (Angle)(5.729577951)maxXAngle,
                        (Angle)(5.729577951)maxYAngle,
                      }),
  (std::vector<SpecialActionBalanceEntry>)specialActionBalanceEntries,
});

MODULE(MotionCombinator)
REQUIRES(OdometryCorrectionTables)
REQUIRES(FallDownState)
REQUIRES(FrameInfo)
REQUIRES(HeadJointRequest)
REQUIRES(InertiaSensorData)
REQUIRES(RobotModel)
REQUIRES(JointData)
REQUIRES(KickEngineOutput)
REQUIRES(MotionSelection)
REQUIRES(SpecialActionsOutput)
REQUIRES(RobotInfo)
REQUIRES(HardnessSettings)
REQUIRES(WalkingEngineOutput)
REQUIRES(WalkingInfo)
REQUIRES(SpeedInfo)
REQUIRES(OrientationData)
REQUIRES(JointRequest)
PROVIDES_WITH_MODIFY_AND_OUTPUT(JointRequest)
PROVIDES_WITH_MODIFY(MotionInfo)
PROVIDES_WITH_MODIFY_AND_OUTPUT(OdometryData)
LOADS_PARAMETER(bool, emergencyOffEnabled)
LOADS_PARAMETER(unsigned, recoveryTime) /**< The number of frames to interpolate after emergency-stop. */
LOADS_PARAMETER(bool, useBalancing)     // use yOffset and balanceParams for balancing upper body
LOADS_PARAMETER(float, yOffset)         // upper body y offset (for asymmetry)
LOADS_PARAMETER(bool, usePlayDeadBalancing)
LOADS_PARAMETER(Angle, hipPitch_playDead)
LOADS_PARAMETER(float, angleY_playDead)
LOADS_PARAMETER(int, hardness_playDead)
LOADS_PARAMETER(BalanceParameters, balanceParamsWalk)
LOADS_PARAMETER(BalanceParameters, balanceParams)
LOADS_PARAMETER(SpecialActionBalanceList, specialActionBalanceList)
LOADS_PARAMETER(bool, correctOdometry)
END_MODULE

class MotionCombinator : public MotionCombinatorBase {
private:
  NonArmeMotionEngineOutput theNonArmeMotionEngineOutput;

  JointData lastJointAngles;    /**< The measured joint angles the last time when not interpolating. */
  OdometryData odometryData;    /**< The odometry data. */
  MotionInfo motionInfo;        /**< Information about the motion currently executed. */
  Pose2D specialActionOdometry; /**< workaround for accumulating special action odometry. */

  unsigned currentRecoveryTime;

  bool headJawInSavePosition;
  bool headPitchInSavePosition;
  bool isFallingStarted;
  unsigned fallingFrame;

  // for balancing
  float lastComX;
  SpecialActionRequest::SpecialActionID lastBalancedSpecialAction;
  unsigned timeWhenSpecialActionStarted;
  bool wasInBalance;

  OdometryData lastOdometryData;
  JointRequest lastJointRequest;

public:
  /**
   * Default constructor.
   */
  MotionCombinator();

private:
  void update(OdometryData& odometryData);
  void update(JointRequest& jointRequest);
  void update(MotionInfo& motionInfo) { motionInfo = this->motionInfo; }

  void saveFall(JointRequest& JointRequest);
  void centerHead(JointRequest& JointRequest);
  void centerArms(JointRequest& jointRequest);
  void centerArm(JointRequest& jointRequest, bool left);
  void balanceUpperBody(JointRequest& jointRequest);

  /**
   * The method copies all joint angles from one joint request to another,
   * but only those that should not be ignored.
   * @param source The source joint request. All angles != JointData::ignore will be copied.
   * @param target The target joint request.
   */
  void copy(const JointRequest& source,
            JointRequest& target,
            const JointData::Joint startJoint = static_cast<JointData::Joint>(0),
            const JointData::Joint endJoint = static_cast<JointData::Joint>(JointData::numOfJoints - 1)) const;

  /**
   * The method interpolates between two joint requests.
   * @param from The first source joint request. This one is the starting point.
   * @param to The second source joint request. This one has to be reached over time.
   * @param fromRatio The ratio of "from" in the target joint request.
   * @param target The target joint request.
   * @param interpolateStiffness Whether to interpolate stiffness.
   */
  void interpolate(const JointRequest& from,
                   const JointRequest& to,
                   float fromRatio,
                   JointRequest& target,
                   bool interpolateStiffness,
                   const JointData::Joint startJoint = static_cast<JointData::Joint>(0),
                   const JointData::Joint endJoint = static_cast<JointData::Joint>(JointData::numOfJoints - 1)) const;
};
