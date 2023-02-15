/**
 * @file Walk2022Generator.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/MotionControl/PlannedSteps.h"

#include "Representations/MotionControl/WalkGeneratorData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/Sensing/GroundContactState.h"
#include "StepTrajectoryGenerator.h"

#include "Core/Module/Module.h"
#include "Core/Debugging/Debugging.h"
#include "Core/Range.h"
#include "Core/Math/Vector.h"

STREAMABLE(
  Walk2022GeneratorCommon,
  {
    ,
    (float)footLiftFirstStepFactor,       /**< Lifting of first step is changed by this factor. */
    (Rangef)supportSwitchPhaseRange,      /**< In which range of the walk phase can the support foot change? */
    (int)maxWeightShiftMisses,            /**< The maximum number of weight shift misses before emergency behavior. */
    (float)emergencyStepSize,             /**< The size of emergency sideways steps in mm. */
    (float)insideTurnRatio,               /**< How much of rotation is done by turning feet to the inside (0..1)? */
    (Pose2D)odometryScale,                /**< Scale measured speeds so that they match the executed speeds. */
    (int)walkStiffness,                   /**< Joint stiffness while walking in %. */
    (Angle)armShoulderRoll,               /**< Arm shoulder angle in radians. */
    (float)armShoulderRollIncreaseFactor, /**< Factor between sideways step size (in m) and additional arm roll angles. */
    (float)armShoulderPitchFactor,        /**< Factor between forward foot position (in m) and arm pitch angles. */
    (float)comTiltForwardIncreaseFactor,  /**< The factor the torso is additionally tilted based on the signed forward step
                                             size when walking forwards. */
    (float)comTiltBackwardIncreaseFactor, /**< The factor the torso is additionally tilted based on the signed forward step
                                             size when walking backwards. */
    (float)targetModeSpeedFactor, /**< Ratio between distance to target and speed to walk with if it cannot be reached in a
                                     single step. */
    (int)numOfComIterations,      /**< Number of iterations for matching the default COM. */
    (float)pComFactor,            /**< Proportional factor for matching the default COM. */
    (float)comTiltFactor,         /**< Factor between the correct torso tilt and the one actually used. */
    (int)standStiffnessDelay,     /**< The time in stand before the stiffness is lowered (in ms). */
    (int)maxWeightShiftSteps, /**< So many emergency steps are allowed in a row, until the robot shall go into standing, to
                                 break the loop. */
    (float)emergencyStepHeightFactor,    /** During an emergency step, the step height is multiplied by this factor. */
    (int)resetEmergencyStepCounter,      /**< If after so many support leg switches no new emergency step was done, the
                                            WeightShiftStepsCounter shall be reseted. */
    (Angle)fastStartStepMaxTurn,         /**< Skip the starting step if the commanded turn speed is lower than this value. */
    (float)fastStartStepMaxSoleDistance, /**< Skip the starting step if the soles of both legs are not further away than this
                                            value. After a long range kick a fast step is usually not possible.*/
    (Rangef)fastStartStepSupportSwitchPhaseRange, /**< In which range of the walk phase can the support foot change, when
                                                     doing the first step after standing? */
    (Pose2D)thresholdStopStandTransition,      /**< Threshold to be able to switch from walk state stopping to standing. */
    (Angle)useMaxTurnSpeedForClampWalk,        /**< Use this max turn speed for clipping forward and left walk values. */
    (bool)useFootSupportSwitchPrediction,      /**< Use predicted support foot switches. */
    (Angle)turnThresholdFootSupportPrediction, /**< Use predicted support foot switches as long as the request turn speed is
                                                  lower than this threshold. */
  });

STREAMABLE(Walk2022GeneratorParams,
           {
             ,
             (float)sideForwardMaxSpeed,                 /**< Use this forward speed, when walking sideways. */
             (float)triggerForwardBoostByThisLeftAmount, /**< Use increased forward speed when walking sideways with
                                                                           minimum this speed (in mm). */
             (Pose2D)maxSpeed,                           /**< Maximum speeds in mm/s and degrees/s. */
             (float)maxSpeedBackwards,                   /**< Maximum backwards speed. Positive, in mm/s. */
             (float)baseWalkPeriod,   /**< Duration of a single step, i.e. half of a walk cycle (in ms). */
             (float)walkHipHeight,    /**< Walk hip height above ankle joint in mm - seems to work from 200 mm to 235 mm. */
             (float)torsoOffset,      /**< The base forward offset of the torso relative to the ankles in mm. */
             (float)gyroLowPassRatio, /**< To which ratio keep old gyro measurements? */
             (float)gyroForwardBalanceFactor,  /**< How much are gyro measurements added to ankle joint angles to
                                                                 compensate falling forwards while walking? */
             (float)gyroBackwardBalanceFactor, /**< How much are gyro measurements added to ankle joint angles to
                                                                 compensate falling backwards while walking? */
             (float)gyroSidewaysBalanceFactor, /**< How much are gyro measurements added to ankle joint angles to
                                                                 compensate falling sideways while standing? */
             (float)maxSpeedDivider,
             (float)stepWidthOffset,
           });

MODULE(Walk2022Generator)
USES(RobotPose)
REQUIRES(FrameInfo)
REQUIRES(MassCalibration)
REQUIRES(RobotDimensions)
REQUIRES(RobotInfo)
REQUIRES(JointData)
USES(JointRequest)
REQUIRES(PlannedSteps)
REQUIRES(RobotModel)
REQUIRES(FallDownState)
REQUIRES(ArmMotionRequest)
REQUIRES(GroundContactState)
REQUIRES(InertiaSensorData)
REQUIRES(FootSupport)
REQUIRES(WalkGeneratorData)
PROVIDES_WITH_MODIFY(WalkGeneratorData)
PROVIDES_WITH_MODIFY(PlannedSteps)
LOADS_PARAMETER(Walk2022GeneratorParams, theWalk2022GeneratorParams)
END_MODULE

class Walk2022Generator : public Walk2022GeneratorBase, public Walk2022GeneratorCommon {

  Pose2D currentSpeed;

  Pose3D leftFoot;
  Pose3D rightFoot;

  Vector3<> leftFootOffset;
  Vector3<> rightFootOffset;

  // Vector3<> leftFootOffset0;
  // Vector3<> rightFootOffset0;

  Pose3D measuredLeftFoot;
  Pose3D measuredRightFoot;

  Angle turnRL;
  // Angle turnRL0;

  // float maxFootHeight;  /**< Maximum foot height in current step (in m). */
  // float maxFootHeight0; /**< Maximum foot height in previous step (in m). */
  // float switchPhase;    /**< The walk phase when the support changed. */
  std::vector<std::pair<float, Vector3<>>> leftTraj;
  std::vector<std::pair<float, Vector3<>>> rightTraj;

  Angle filteredGyroX = 0_deg; /**< Lowpass-filtered gyro measurements around y axis (in radians/s). */
  Angle filteredGyroY = 0_deg; /**< Lowpass-filtered gyro measurements around y axis (in radians/s). */

  Vector3<> prevLeftFootOffset;
  Vector3<> prevRightFootOffset;

  Angle prevTurn;                  /**< The value of "turn" in the previous cycle. For odometry calculation. */
  Angle torsoTilt;                 /**< The current tilt of the torso (in radians). */
  unsigned timeWhenStandBegan = 0; /**< The time when stand began (in ms). */

  float footSoleDistanceAtStepStart; /**< The distance of both feet at the start of the step. */
  float forwardTiltOffset = 0;

  void update(WalkGeneratorData& generator);

  void
  calcJoints(WalkGeneratorData& generator, const Pose2D& speed, const Pose2D& target, WalkGeneratorData::WalkMode walkMode);
  void reset(WalkGeneratorData& generator);
  Pose2D calcOdometryOffset(WalkGeneratorData& generator, bool isLeftSwingFoot);
  void compensateArmPosition(const Pose3D& leftFoot, const Pose3D& rightFoot, JointRequest& jointRequest);
  void stateCheck(WalkGeneratorData& generator, Vector3<> leftFootOffset, Vector3<> rightFootOffset, float turnRL);

public:
  Walk2022Generator();
};
