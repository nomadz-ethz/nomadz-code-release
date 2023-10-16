/**
 * @file Walk2014Generator.h
 *
 * This file declares the UNSW 2014 walk generator. It was refactored to fit into the B-Human framework.
 *
 * This file is subject to the terms of the BHuman 2017 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are Bernhard Hengst and Thomas Röfer
 */

/**
 * Copyright 2010 The University of New South Wales (UNSW).
 *
 * This file is part of the 2010 team rUNSWift RoboCup entry. You may
 * redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version as
 * modified below. As the original licensors, we add the following
 * conditions to that license:
 *
 * In paragraph 2.b), the phrase "distribute or publish" should be
 * interpreted to include entry into a competition, and hence the source
 * of any derived work entered into a competition must be made available
 * to all parties involved in that competition under the terms of this
 * license.
 *
 * In addition, if the authors of a derived work publish any conference
 * proceedings, journal articles or other academic papers describing that
 * derived work, then appropriate academic citations to the original work
 * must be included in that publication.
 *
 * This rUNSWift source is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this source code; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#pragma once

#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/MotionControl/Walk2014Modifier.h"
#include "Representations/MotionControl/WalkGeneratorData.h"
#include "Representations/MotionControl/WalkLearner.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Core/Module/Module.h"
#include "Core/Range.h"

STREAMABLE(
  Walk2014GeneratorCommon,
  {
    ,
    (Vector2f)maxAcceleration, /**< Maximum acceleration of forward and sideways speed at each leg change to ratchet up/down
                                  in (mm/s/step). */
    (Vector2f)maxDeceleration, /**< (Positive) maximum deceleration of forward and sideways speed at each leg change to
                                  ratchet up/down in (mm/s/step). */
    (Pose2D)slowMaxSpeed,      /**< Maximum speeds in mm/s and degrees/s. Slower for demo games. */
    (float)slowMaxSpeedBackwards,  /**< Maximum backwards speed. Positive, in mm/s. Slower for demo games. */
    (Vector2f)slowMaxAcceleration, /**< Maximum acceleration of forward and sideways speed at each leg change to ratchet
                                      up/down in (mm/s/step). Slower for demo games. */
    (float)walkVolumeTranslationExponent,     /**< This affects the relationship between forward and sideways. */
    (float)walkVolumeRotationExponent,        /**< Higher value allows turn to be higher with a high translation. */
    (float[2])footLiftIncreaseFactorForwards, /**< Additional lifting as factor of forward speed for left and right foot. */
    (float)footLiftIncreaseFactorSidewards,   /**< Additional lifting as factor of sideways speed. */
    (float)footLiftIncreaseFactorBackwards,   /**< Additional lifting as factor of backward speed. */
    (float)footLiftFirstStepFactor,           /**< Lifting of first step is changed by this factor. */
    (Rangef)supportSwitchPhaseRange,          /**< In which range of the walk phase can the support foot change? */
    (int)maxWeightShiftMisses,                /**< The maximum number of weight shift misses before emergency behavior. */
    (float)emergencyStepSize,                 /**< The size of emergency sideways steps in mm. */
    (float)minSlowWeightShiftRatio,           /**< How much longer than expected is a slow weight shift? */
    (int)maxSlowWeightShifts,                 /**< How many slow weight shifts are acceptable? */
    (int)slowWaitShiftStandDelay,             /**< How long to stand after slow weight shifts were detected (in ms)? */
    (float)insideTurnRatio,                   /**< How much of rotation is done by turning feet to the inside (0..1)? */
    (Pose2D)odometryScale,                    /**< Scale measured speeds so that they match the executed speeds. */
    (int)walkStiffness,                       /**< Joint stiffness while walking in %. */
    (Angle)armShoulderRoll,                   /**< Arm shoulder angle in radians. */
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

STREAMABLE(
  Walk2014GeneratorParams,
  {
    ,
    (float)sideForwardMaxSpeed,                 /**< Use this forward speed, when walking sideways. */
    (float)triggerForwardBoostByThisLeftAmount, /**< Use increased forward speed when walking sideways with
                                                                  minimum this speed (in mm). */
    (Pose2D)maxSpeed,                           /**< Maximum speeds in mm/s and degrees/s. */
    (float)maxSpeedBackwards,                   /**< Maximum backwards speed. Positive, in mm/s. */
    (float)baseWalkPeriod,                      /**< Duration of a single step, i.e. half of a walk cycle (in ms). */
    (float)sidewaysWalkPeriodIncreaseFactor,    /**< Additional duration when walking sideways at maximum speed (in ms). */
    (float)forwardsWalkPeriodIncreaseFactor,
    (float)walkHipHeight,             /**< Walk hip height above ankle joint in mm - seems to work from 200 mm to 235 mm. */
    (float)baseFootLift,              /**< Base foot lift in mm. */
    (float)torsoOffset,               /**< The base forward offset of the torso relative to the ankles in mm. */
    (float)gyroLowPassRatio,          /**< To which ratio keep old gyro measurements? */
    (float)gyroForwardBalanceFactor,  /**< How much are gyro measurements added to ankle joint angles to
                                                        compensate falling forwards while walking? */
    (float)gyroBackwardBalanceFactor, /**< How much are gyro measurements added to ankle joint angles to
                                                        compensate falling backwards while walking? */
    (float)gyroSidewaysBalanceFactor, /**< How much are gyro measurements added to ankle joint angles to
                                                        compensate falling sideways while standing? */
    (float)maxSpeedDivider,
    (float)stepWidthOffset,
    (float)maxLandingAngle,
    (float)landingAngleScaler,
  });

MODULE(Walk2014Generator)
REQUIRES(FootSupport)
REQUIRES(FrameInfo)
REQUIRES(FallDownState)
REQUIRES(GroundContactState)
REQUIRES(InertiaSensorData)
REQUIRES(JointData)
REQUIRES(ArmMotionRequest)
USES(JointRequest)
REQUIRES(MassCalibration)
REQUIRES(RobotDimensions)
REQUIRES(RobotInfo)
REQUIRES(RobotModel)
REQUIRES(Walk2014Modifier)
REQUIRES(WalkLearner)
REQUIRES(WalkGeneratorData)
PROVIDES_WITH_MODIFY(WalkGeneratorData)
LOADS_PARAMETER(Walk2014GeneratorParams, theWalk2014GeneratorParams)
END_MODULE

class Walk2014Generator : public Walk2014GeneratorBase, public Walk2014GeneratorCommon {
  enum WalkState { standing, starting, walking, stopping } walkState; /**< The current state of the engine. */

  const float mmPerM = 1000.f;
  float forward;        /**< Forward speed in m/step. Forward is positive. */
  float lastForward;    /**< The forward speed of the previous step. */
  float forwardL;       /**< The forward offset of the left foot (in m). */
  float forwardR;       /**< The forward offset of the right foot (in m). */
  float forwardL0;      /**< Forward offset of the left foot when the support changed (in m). */
  float forwardR0;      /**< Forward offset of the right foot when the support changed (in m). */
  float left;           /**< Sideways speed in m/step. Left is positive. */
  float lastLeft;       /**< Sideways speed in for previous step m/s. Left is positive. */
  Angle leftL;          /**< The sideways angle of the left foot (in radians). */
  Angle leftR;          /**< The sideways angle of the right foot (in radians). */
  Angle turn;           /**< Turn speed in radians/step. Anti-clockwise is positive. */
  Angle turnRL;         /**< The turn angle for both feet (in radians). */
  Angle turnRL0;        /**< The turn angle for both feet when the support changed (in radians). */
  Angle swingAngle;     /**< Recovery angle for side stepping (in radians). */
  float maxFootHeight;  /**< Maximum foot height in current step (in m). */
  float maxFootHeight0; /**< Maximum foot height in previous step (in m). */
  float switchPhase;    /**< The walk phase when the support changed. */
  std::vector<Vector3<>> leftTraj;
  std::vector<Vector3<>> rightTraj;
  enum {
    weightDidShift,
    weightDidNotShift,
    emergencyStep
  } weightShiftStatus;                           /**< Has the weight shifted when the previous step ended? */
  unsigned timeWhenSlowWeightShiftsDetected = 0; /**< The time when slow weight shifts were detected. */
  Angle filteredGyroX = 0_deg;                   /**< Lowpass-filtered gyro measurements around y axis (in radians/s). */
  Angle filteredGyroY = 0_deg;                   /**< Lowpass-filtered gyro measurements around y axis (in radians/s). */
  float prevForwardL;                /**< The value of "forwardL" in the previous cycle. For odometry calculation. */
  float prevForwardR;                /**< The value of "forwardR" in the previous cycle. For odometry calculation. */
  Angle prevLeftL;                   /**< The value of "leftL" in the previous cycle. For odometry calculation. */
  Angle prevLeftR;                   /**< The value of "leftR" in the previous cycle. For odometry calculation. */
  Angle prevTurn;                    /**< The value of "turn" in the previous cycle. For odometry calculation. */
  int weightShiftMisses;             /**< How often was the weight not shifted in a row? */
  int slowWeightShifts;              /**< How often took the weight shift significantly longer in a row? */
  Angle torsoTilt;                   /**< The current tilt of the torso (in radians). */
  unsigned timeWhenStandBegan = 0;   /**< The time when stand began (in ms). */
  int emergencyStepCounter;          /**< How many emergency steps were done? */
  int noEmergencyStepCounter;        /**< How many steps are done without an emergency step? */
  float footSoleDistanceAtStepStart; /**< The distance of both feet at the start of the step. */
  bool emergencyLift = false;        /**< Lift the foot higher during the current step. */
  float forwardTiltOffset = 0;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param generator The representation updated.
   */
  void update(WalkGeneratorData& generator) override;

  /**
   * Initializes the generator. Must be called whenever the control is returned to this module after
   * another one was responsible for creating the motions. Must also be called once after creation.
   */
  void reset(WalkGeneratorData& generator);

  /**
   * Calculates a new set of joint angles to let the robot walk or stand. Must be called every 10 ms.
   * @param generator The output of this module.
   * @param speed The speed or step size to walk with. If everything is zero, the robot stands.
   * @param target The target to walk to if in target mode.
   * @param walkMode How are speed and target interpreted?
   */
  void
  calcJoints(WalkGeneratorData& generator, const Pose2D& speed, const Pose2D& target, WalkGeneratorData::WalkMode walkMode);

  /**
   * Calculates the parameters of the next step. Must be called when
   * a new step starts.
   * @param generator The output of this module.
   * @param speed The speed or step size to walk with. If everything is zero, the robot stands.
   * @param target The target to walk to if in target mode.
   * @param walkMode How are speed and target interpreted?
   */
  void calcNextStep(WalkGeneratorData& generator,
                    const Pose2D& speed,
                    const Pose2D& target,
                    WalkGeneratorData::WalkMode walkMode);

  /**
   * The method determines the forward, left, and lift offsets of both feet.
   * The method distinguishes between the swing foot and the support foot.
   * @param swingFootSign A sign based on the swingFoot (1 : left is swing foot, -1 right is swing foot).
   * @param forwardSwing0 Forward offset of the current swing foot when the support changed (in m).
   * @param forwardSupport0 Forward offset of the current support foot when the support changed (in m).
   * @param forwardSwing The new forward offset of the swing foot is returned here (in m).
   * @param forwardSupport The new forward offset of the support foot is returned here (in m).
   * @param leftSwing The new sideways angle of the swing foot is returned here (in radians).
   * @param leftSupport The new sideways angle of the support foot is returned here (in radians).
   * @param footHeightSwing The new lift offset of the swing foot is returned here (in m).
   * @param footHeightSupport The new lift offset of the support foot is returned here (in m).
   */
  void calcFootOffsets(WalkGeneratorData& generator,
                       float swingFootSign,
                       float forwardSwing0,
                       float forwardSupport0,
                       float& forwardSwing,
                       float& forwardSupport,
                       Angle& leftSwing,
                       Angle& leftSupport,
                       float& footHeightSwing,
                       float& footHeightSupport);

  /**
   * Determines the motion of the robot since the previous frame.
   * @param isLeftSwingFoot Is the left foot the current swing foot?
   * @return The offset in mm and radians.
   */
  Pose2D calcOdometryOffset(WalkGeneratorData& generator, bool isLeftSwingFoot);

  /**
   * Return a measure for how "big" the requested motion is, i.e. the "walk volume".
   * This is used to limit the requested motion to keep the steps executable.
   * @param forward Forward speed as a ratio of the maximum forward speed.
   * @param left Sideways speed as a ratio of the maximum sideways speed.
   * @param turn Turn speed as a ratio of the maximum turn speed.
   * @return The walk volume.
   */
  float calcWalkVolume(float forward, float left, float turn) const;

  /**
   * Limit the requested motion to keep the steps executable. The request
   * is clamped to the surface of an ellipsoid.
   * @param maxSpeed The maximum speeds allowed.
   * @param maxSpeedBackwards The maximum speed when walking backwards.
   * @param forward The forward speed in mm/s will be clamped im necessary.
   * @param left Sideways speed in mm/s will be clamped im necessary.
   * @param turn Turn speed in radians/s will be clamped im necessary.
   * @return Were the parameters actually clamped?
   */
  bool ellipsoidClampWalk(const Pose2D& maxSpeed, float maxSpeedBackwards, float& forward, float& left, float& turn) const;

  /**
   * Returns values on a parabola with f(0) = f(1) = 0, f(0.5) = 1.
   * @param f A value between 0 and 1.
   * @return The value on the parabola for "f".
   */
  float parabolicReturn(float) const;

  /**
   * Returns values on a parabola with f(0) = 0, f(period) = 1.
   * @param time A value between 0 and "period".
   * @param period The duration of a period.
   * @return The value on the parabola for "time".
   */
  float parabolicStep(float time, float period) const;

  /**
   * Compensates for the changed position of the COM resulting from arm motion.
   * The torso is tilted to move the COM.
   * @param leftFoot The pose of the left foot's sole relative to the torso.
   * @param rightFoot The pose of the right foot's sole relative to the torso.
   * @param jointRequest The joint request as determined by the walk generator.
   *                     The joint request is changed to compensate for the
   *                     effect of external arm movements.
   */
  void compensateArmPosition(const Pose3D& leftFoot, const Pose3D& rightFoot, JointRequest& jointRequest);

public:
  /** The constructor loads the common parameters. */
  Walk2014Generator();
};
