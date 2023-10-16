/**
 * @file Walk2022Generator.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/OrientationData.h"
#include "Representations/MotionControl/PlannedSteps.h"
#include "Representations/MotionControl/ArmMotionEngineOutput.h"

#include "Representations/MotionControl/WalkGeneratorData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/GroundContactState.h"
#include "StepTrajectoryGenerator.h"
#include "OscillationController.h"
#include "ZMPController.h"

#include "Core/Module/Module.h"
#include "Core/Debugging/Debugging.h"
#include "Core/Range.h"
#include "Core/Math/Vector.h"

STREAMABLE(
  Walk2022GeneratorParams,
  {
    ,
    (float)footLiftFirstStepFactor,       /**< Lifting of first step is changed by this factor. */
    (int)walkStiffness,                   /**< Joint stiffness while walking in %. */
    (Angle)armShoulderRoll,               /**< Arm shoulder angle in radians. */
    (float)armShoulderRollIncreaseFactor, /**< Factor between sideways step size (in m) and additional arm roll angles. */
    (float)armShoulderPitchFactor,        /**< Factor between forward foot position (in m) and arm pitch angles. */
    (float)comTiltFactor,                 /**< Factor between the correct torso tilt and the one actually used. */
    (int)standStiffnessDelay,             /**< The time in stand before the stiffness is lowered (in ms). */
    (Rangef)supportSwitchPhaseRange,      /**< In which range of the walk phase can the support foot change? */

    (Pose2D)thresholdStopStandTransition, /**< Threshold to be able to switch from walk state stopping to standing. */
    (int)numOfComIterations,              /**< Number of iterations for matching the default COM. */

    (Rangef)stepCorrectionAllowPhaseRange,
    (float)oscillationTriggerStrength,
    (Vector3<>)originOffset,
    (float)amplitudeOriginSwingY,
    (bool)useBezierCurve,
    (float)sideStepOriginOffset_P,
    (float)walkHipHeight,    /**< Walk hip height above ankle joint in mm - seems to work from 200 mm to 235 mm. */
    (float)torsoOffsetBase,  /**< The base forward offset of the torso relative to the ankles in mm. */
    (float)gyroLowPassRatio, /**< To which ratio keep old gyro measurements? */
    (float)gyroForwardBalanceFactor,
    (float)gyroBackwardBalanceFactor,
    (float)gyroSidewaysBalanceFactor,
    (float)baseFootLift,
    (Pose2D)footLiftIncreaseFactor,
    (bool)enableZMP,
    (float)additionalYOffset,
    (float)augmentComRef,
    (float)gyroCompensationFactorP,
    (float)gyroCompensationFactorD,
    (float)shiftPlotPointVertical,
    (float)deltaScaler,
    (bool)enableGyroCompensation,
  });

MODULE(Walk2022Generator)
USES(RobotPose)
REQUIRES(FrameInfo)
REQUIRES(MassCalibration)
REQUIRES(RobotDimensions)
REQUIRES(RobotInfo)
REQUIRES(FilteredJointData)
USES(JointRequest)
REQUIRES(PlannedSteps)
REQUIRES(MotionRequest)
REQUIRES(RobotModel)
REQUIRES(FallDownState)
REQUIRES(ArmMotionRequest)
REQUIRES(GroundContactState)
REQUIRES(InertiaSensorData)
REQUIRES(OrientationData)
REQUIRES(FootSupport)
REQUIRES(WalkGeneratorData)
REQUIRES(ArmMotionEngineOutput)
PROVIDES_WITH_MODIFY(WalkGeneratorData)
END_MODULE

class Walk2022Generator : public Walk2022GeneratorBase, public Walk2022GeneratorParams {

  Pose2D requestedSpeed;
  Pose2D requestedTarget;
  WalkGeneratorData::WalkMode requestedWalkMode;

  Pose2D currentPlannedSpeed; // The desired speed at the end of the step execution.
  Pose2D prevPlannedSpeed;
  Pose2D currentSpeed; // The current speed. In other words the relative speed of the origin.
  Pose2D newTargetGait[Leg::numOfSides];
  Pose3D measuredFoot[Leg::numOfSides];

  Pose2D measuredFootPos[Leg::numOfSides];
  Pose2D measuredPrevFootPos[Leg::numOfSides];

  std::vector<std::pair<float, Vector3<>>> debugTraj[Leg::numOfSides];

  Angle filteredGyroX = 0_deg; /**< Lowpass-filtered gyro measurements around y axis (in radians/s). */
  Angle filteredGyroY = 0_deg; /**< Lowpass-filtered gyro measurements around y axis (in radians/s). */

  Angle torsoTilt;                 /**< The current tilt of the torso (in radians). */
  unsigned timeWhenStandBegan = 0; /**< The time when stand began (in ms). */

  float forwardTiltOffset = 0;
  float correctionPhase = 0;
  bool correctionApplied = false;

  Vector3<> targetOriginOffset;
  float torsoOffset;
  float torsoOffsetTarget;
  Vector3<> prevTargetOriginOffset;
  Leg::Side swingSide;
  Leg::Side supportSide;
  const StepTraj::TrajType finalTrajectoryType = StepTraj::currentOffset;
  OscillationController oscillationController;
  ZMPController zmpController;
  Pose2D globPosAtGaitSwitch;
  Rangef singleSupportPhase;
  Rangef doubleSupportPhase;
  std::vector<Vector2<>> offsetFromDefault;
  Vector2<> currentComRef;
  Vector2<> currentZMP;
  Vector2<> originRef;
  Vector2<> gyroCorrectionStates[3]; // Pos, Vel, Acc
  Pose3D prevFootFinal[2];
  Pose3D augmentedFootFinal[2];

  void reset(WalkGeneratorData& generator);
  void update(WalkGeneratorData& generator) override;
  void updateSettings(WalkGeneratorData& generator,
                      Pose2D& reqSpeed,
                      Pose2D& reqTarget,
                      WalkGeneratorData::WalkMode requestedWalkMode);
  void recomputeOptions(WalkGeneratorData& generator);
  void calcTraj(WalkGeneratorData& generator);
  void generateOriginOffset(WalkGeneratorData& generator);
  void calcZMPOffset(WalkGeneratorData& generator);
  void calcGyroOffset(WalkGeneratorData& generator);
  void updateLegJoints(WalkGeneratorData& generator);
  void calcNextStep(WalkGeneratorData& generator);
  void calcJoints(WalkGeneratorData& generator,
                  Pose2D& reqSpeed,
                  Pose2D& reqTarget,
                  WalkGeneratorData::WalkMode requestedWalkMode);
  void calcOdometryOffset(WalkGeneratorData& generator);
  void updateArmJoints(WalkGeneratorData& generator);
  void compensateArmPosition(WalkGeneratorData& generator);
  void setStiffness(WalkGeneratorData& generator);
  float calcStepHeight();
  void applyGyroBalance(WalkGeneratorData& generator);
  void checkSwitchCondition(WalkGeneratorData& generator);
  void stateCheck(WalkGeneratorData& generator);
  void customDebugDrawing(WalkGeneratorData& generator);

public:
  Walk2022Generator();
};
