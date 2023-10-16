/**
 * @file Walk2014Generator.cpp
 *
 * The period of each foot-step is set by T. T generates the forcing function by alternatively lifting each foot.
 * Control:
 * The change of support foot is driven by the ZMP switching sign, but must be > T/2. If > 3*T we switch to try
 * to revive.
 * The front-back sway is controlled by ankle tilts proportional to the y gyro.
 * The user specifies forward(m), left(m), turn(radians) to activate the walk.
 * If these values are all zero the robot stands.
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

#include <algorithm>

#include "Core/Math/Rotation.h"

#include "Walk2014Generator.h"
#include "Core/System/BHAssert.h"
#include "Core/System/SystemCall.h"
#include "Core/Math/Constants.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Motion/ForwardKinematic.h"
#include "Core/Range.h"
#include "Core/Streams/InStreams.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Debugging/DebugDrawings3D.h"

#include "Representations/Sensing/RobotModel.h"

MAKE_MODULE(Walk2014Generator, Motion Control);

Walk2014Generator::Walk2014Generator() {
  InMapFile stream("walk2014GeneratorCommon.cfg");
  if (stream.exists()) {
    stream >> static_cast<Walk2014GeneratorCommon&>(*this);
  }
}

void Walk2014Generator::update(WalkGeneratorData& generator) {
  DECLARE_PLOT("module:Walk2014Generator:leftFootHightCurrent");
  DECLARE_PLOT("module:Walk2014Generator:leftFootHightDesired");
  DECLARE_PLOT("module:Walk2014Generator:rightFootHightCurrent");
  DECLARE_PLOT("module:Walk2014Generator:rightFootHightDesired");
  DECLARE_PLOT("module:Walk2014Generator:leftFootForwardCurrent");
  DECLARE_PLOT("module:Walk2014Generator:leftFootForwardDesired");
  DECLARE_PLOT("module:Walk2014Generator:rightFootForwardCurrent");
  DECLARE_PLOT("module:Walk2014Generator:rightFootForwardDesired");
  DECLARE_PLOT("module:Walk2014Generator:leftFootSideCurrent");
  DECLARE_PLOT("module:Walk2014Generator:leftFootSideDesired");
  DECLARE_PLOT("module:Walk2014Generator:rightFootSideCurrent");
  DECLARE_PLOT("module:Walk2014Generator:rightFootSideDesired");
  DECLARE_PLOT("module:Walk2014Generator:balanceAdjustmentPitch");
  DECLARE_PLOT("module:Walk2014Generator:balanceAdjustmentRoll");
  DECLARE_PLOT("module:Walk2014Generator:imbalanceX");
  DECLARE_PLOT("module:Walk2014Generator:imbalanceY");

  DECLARE_PLOT("module:Walk2014Generator:debug1");
  DECLARE_PLOT("module:Walk2014Generator:debug2");

  DECLARE_DEBUG_DRAWING3D("module:Walk2014Generator:nextStep", "robot");

  MODIFY("parameters:Walk2014Generator:params", theWalk2014GeneratorParams);
  MODIFY("parameters:Walk2014Generator:common", static_cast<Walk2014GeneratorCommon&>(*this));

  // Use other parameters in demo games to take care of the robots

  useMaxTurnSpeedForClampWalk =
    std::min(static_cast<float>(useMaxTurnSpeedForClampWalk), theWalk2014GeneratorParams.maxSpeed.rotation);

  generator.reset = [this, &generator]() { reset(generator); };

  generator.calcJoints = [this,
                          &generator](const Pose2D& speed, const Pose2D& target, WalkGeneratorData::WalkMode walkMode) {
    calcJoints(generator, speed, target, walkMode);
  };

  generator.maxSpeed = Pose2D(theWalk2014GeneratorParams.maxSpeed.rotation * odometryScale.rotation,
                              theWalk2014GeneratorParams.maxSpeed.translation.x * odometryScale.translation.x,
                              theWalk2014GeneratorParams.maxSpeed.translation.y * odometryScale.translation.y);

  filteredGyroX = theWalk2014GeneratorParams.gyroLowPassRatio * filteredGyroX +
                  (1.f - theWalk2014GeneratorParams.gyroLowPassRatio) * theInertiaSensorData.gyro.x;
  filteredGyroY = theWalk2014GeneratorParams.gyroLowPassRatio * filteredGyroY +
                  (1.f - theWalk2014GeneratorParams.gyroLowPassRatio) * theInertiaSensorData.gyro.y;

  // Decide if the gyroBalanceFactors shall be taken from the WalkLearner.
  if (theWalk2014Modifier.numOfGyroPeaks > 0) {
    theWalkLearner.setBaseWalkParams(theWalk2014GeneratorParams.gyroForwardBalanceFactor,
                                     theWalk2014GeneratorParams.gyroBackwardBalanceFactor,
                                     generator.speed.translation.x);
    if (theWalkLearner.newGyroBackwardBalance > 0) {
      theWalk2014GeneratorParams.gyroBackwardBalanceFactor = theWalkLearner.newGyroBackwardBalance;
    }
    if (theWalkLearner.newGyroForwardBalance > 0) {
      theWalk2014GeneratorParams.gyroForwardBalanceFactor = theWalkLearner.newGyroForwardBalance;
    }
  }
}

void Walk2014Generator::reset(WalkGeneratorData& generator) {
  generator.stepDuration = 0.f;
  generator.t = 0.f;
  generator.speed = Pose2D();
  generator.upcomingOdometryOffset = Pose2D();
  walkState = standing;
  timeWhenStandBegan = theFrameInfo.time;
  if (theRobotInfo.penalty != PENALTY_NONE && theRobotInfo.penalty != PENALTY_SPL_ILLEGAL_MOTION_IN_SET) {
    filteredGyroX = filteredGyroY = 0;
  }

  forward = lastForward = 0.f;
  forwardL = forwardL0 = 0.f;
  forwardR = forwardR0 = 0.f;
  left = lastLeft = 0.f;
  leftL = leftR = 0_deg;
  turnRL = turnRL0 = 0_deg;
  swingAngle = 0_deg;
  switchPhase = 0.f;
  maxFootHeight = maxFootHeight0 = 0.f;
  weightShiftStatus = weightDidNotShift;
  prevForwardL = prevForwardR = 0.f;
  prevLeftL = prevLeftR = 0_deg;
  prevTurn = 0_deg;
  weightShiftMisses = 0;
  slowWeightShifts = 0;
  torsoTilt = 0_deg;
  emergencyStepCounter = 0;
  noEmergencyStepCounter = 0;
  footSoleDistanceAtStepStart = std::fabs(theRobotModel.soleLeft.translation.y - theRobotModel.soleRight.translation.y);
}

void Walk2014Generator::calcJoints(WalkGeneratorData& generator,
                                   const Pose2D& speed,
                                   const Pose2D& target,
                                   WalkGeneratorData::WalkMode walkMode) {

  // 1. Read in new walk values (forward, left, turn, power) only at the start of a walk step cycle, ie when t = 0
  if (generator.t == 0) {
    calcNextStep(generator, speed, target, walkMode);
    if (generator.isLeftPhase) {
      leftTraj.clear();
      rightTraj.clear();
    }
  }

  // 2. Update timer
  if (generator.t > generator.stepDuration - 1.1 * Constants::motionCycleTime &&
      theFallDownState.state != FallDownState::upright) {
    generator.t += 0;
    if (generator.stepDuration > theWalk2014GeneratorParams.baseWalkPeriod - Constants::motionCycleTime) {
      generator.t -= 0.5 * Constants::motionCycleTime;
      generator.stepDuration -= 0.5 * Constants::motionCycleTime;
    }
  } else {
    generator.t += Constants::motionCycleTime;
  }

  // 3. Determine Walk2014 Option
  if (walkState != standing && forward == 0 && left == 0 && turn == 0) {
    walkState = stopping;
  } else if (walkState != walking && (forward != 0 || left != 0 || turn != 0)) {
    walkState = starting;
  }

  // 5. Determine walk variables throughout the walk step phase
  float footHL = 0;
  float footHR = 0;
  Rangef supportSwitchPhaseRangeUsed(supportSwitchPhaseRange.min, supportSwitchPhaseRange.max);
  if (walkState == standing) {
    footSoleDistanceAtStepStart = std::fabs(theRobotModel.soleLeft.translation.y - theRobotModel.soleRight.translation.y);
    emergencyStepCounter = 0;
    noEmergencyStepCounter = 0;
    generator.stepDuration = generator.t = 0.f;
    footHL = footHR = 0;
  } else {
    // 5.3 Calculate intra-walkphase forward, left and turn at time-step dt
    if (generator.isLeftPhase) {
      calcFootOffsets(generator, 1.f, forwardL0, forwardR0, forwardL, forwardR, leftL, leftR, footHL, footHR);
    } else {
      calcFootOffsets(generator, -1.f, forwardR0, forwardL0, forwardR, forwardL, leftR, leftL, footHR, footHL);
    }

    // 5.4 Special conditions when priming the walk
    // forward check is not needed, because forward is clipped with the maxAcceleration
    // check is used to decide, if the robot can execute a real step as a starting step
    if (walkState == starting &&
        (std::abs(turn) > fastStartStepMaxTurn || footSoleDistanceAtStepStart > fastStartStepMaxSoleDistance)) {
      footHL *= footLiftFirstStepFactor; // reduce max lift due to short duration
      footHR *= footLiftFirstStepFactor;
      forwardL = forwardR = 0; // don't move on starting
      leftR = leftL = 0;
      turnRL = 0;
      generator.speed = Pose2D();
      if (left != 0.f) { // make first real step in direction of movement
        generator.isLeftPhase = left > 0;
      }
    } else if (walkState == starting) {
      weightShiftStatus = weightDidShift;
      supportSwitchPhaseRangeUsed =
        Rangef(fastStartStepSupportSwitchPhaseRange.min, fastStartStepSupportSwitchPhaseRange.max);
      if (left != 0.f) { // make first real step in direction of movement
        generator.isLeftPhase = left > 0;
      }
    }
  }

  // 8. Odometry update for localization
  generator.odometryOffset = calcOdometryOffset(generator, generator.isLeftPhase);

  // 9.1 Foot poses
  Pose3D leftFoot =
    Pose3D(0, theRobotDimensions.yHipOffset + theWalk2014GeneratorParams.stepWidthOffset * forward / 278, 0)
      .rotateX(-leftL)
      .translate(-forwardL * mmPerM - theWalk2014GeneratorParams.torsoOffset,
                 0,
                 -(theWalk2014GeneratorParams.walkHipHeight - theRobotDimensions.heightLeg5Joint - footHL * mmPerM) /
                   std::cos(leftL))
      .rotateX(leftL)
      .rotateZ(turnRL)
      .translate(0, 0, -theRobotDimensions.heightLeg5Joint);
  Pose3D rightFoot =
    Pose3D(0, -theRobotDimensions.yHipOffset - theWalk2014GeneratorParams.stepWidthOffset * forward / 278, 0)
      .rotateX(-leftR)
      .translate(-forwardR * mmPerM - theWalk2014GeneratorParams.torsoOffset,
                 0,
                 -(theWalk2014GeneratorParams.walkHipHeight - theRobotDimensions.heightLeg5Joint - footHR * mmPerM) /
                   std::cos(leftR))
      .rotateX(leftR)
      .rotateZ(-turnRL)
      .translate(0, 0, -theRobotDimensions.heightLeg5Joint);
  Rangef landingAngleRangeX = Rangef(fromDegrees(-theWalk2014GeneratorParams.maxLandingAngle),
                                     fromDegrees(theWalk2014GeneratorParams.maxLandingAngle));
  Rangef landingAngleRangeY = Rangef(fromDegrees(-theWalk2014GeneratorParams.maxLandingAngle),
                                     fromDegrees(theWalk2014GeneratorParams.maxLandingAngle) / 3);

  float swingFootLandingAngleX =
    landingAngleRangeX.limit(speed.translation.y / theWalk2014GeneratorParams.landingAngleScaler);
  float swingFootLandingAngleY =
    landingAngleRangeY.limit(-speed.translation.x / theWalk2014GeneratorParams.landingAngleScaler);
  float t = (generator.stepDuration == 0) ? 0 : generator.t / generator.stepDuration;
  if (generator.isLeftPhase) {
    leftFoot.rotateX(swingFootLandingAngleX * t).rotateY(swingFootLandingAngleY * t);
  } else {
    rightFoot.rotateX(swingFootLandingAngleX * t).rotateY(swingFootLandingAngleY * t);
  }
  PLOT("module:Walk2014Generator:leftFootHightCurrent", theRobotModel.soleLeft.translation.z);
  PLOT("module:Walk2014Generator:leftFootHightDesired", leftFoot.translation.z);
  PLOT("module:Walk2014Generator:rightFootHightCurrent", theRobotModel.soleRight.translation.z);
  PLOT("module:Walk2014Generator:rightFootHightDesired", rightFoot.translation.z);
  PLOT("module:Walk2014Generator:leftFootForwardCurrent", theRobotModel.soleLeft.translation.x);
  PLOT("module:Walk2014Generator:leftFootForwardDesired", leftFoot.translation.x);
  PLOT("module:Walk2014Generator:rightFootForwardCurrent", theRobotModel.soleRight.translation.x);
  PLOT("module:Walk2014Generator:rightFootForwardDesired", rightFoot.translation.x);
  PLOT("module:Walk2014Generator:leftFootSideCurrent", theRobotModel.soleLeft.translation.y);
  PLOT("module:Walk2014Generator:leftFootSideDesired", leftFoot.translation.y);
  PLOT("module:Walk2014Generator:rightFootSideCurrent", theRobotModel.soleRight.translation.y);
  PLOT("module:Walk2014Generator:rightFootSideDesired", rightFoot.translation.y);
  rightTraj.push_back(rightFoot.translation);
  leftTraj.push_back(leftFoot.translation);

  COMPLEX_DRAWING3D("module:Walk2014Generator:nextStep", {
    // Requested Trajectory
    int colorAlphaIndex = 0;
    for (auto& point : rightTraj) {
      SPHERE3D("module:Walk2014Generator:nextStep",
               point.x,
               point.y,
               point.z,
               2,
               ColorRGBA(0, 255, 0, (int)(255 * colorAlphaIndex / rightTraj.size())));
      colorAlphaIndex++;
    }
    colorAlphaIndex = 0;
    for (auto& point : leftTraj) {
      SPHERE3D("module:Walk2014Generator:nextStep",
               point.x,
               point.y,
               point.z,
               2,
               ColorRGBA(0, 255, 0, (int)(255 * colorAlphaIndex / leftTraj.size())));
      colorAlphaIndex++;
    }
    SPHERE3D("module:Walk2014Generator:nextStep",
             rightFoot.translation.x,
             rightFoot.translation.y,
             rightFoot.translation.z,
             3,
             ColorClasses::red);
    SPHERE3D("module:Walk2014Generator:nextStep",
             leftFoot.translation.x,
             leftFoot.translation.y,
             leftFoot.translation.z,
             3,
             ColorClasses::red);

    // Current Measured Position
    SPHERE3D("module:Walk2014Generator:nextStep",
             theRobotModel.limbs[MassCalibration::footRight].translation.x,
             theRobotModel.limbs[MassCalibration::footRight].translation.y,
             theRobotModel.limbs[MassCalibration::footRight].translation.z - theRobotDimensions.heightLeg5Joint,
             3,
             ColorClasses::purple1);
    SPHERE3D("module:Walk2014Generator:nextStep",
             theRobotModel.limbs[MassCalibration::footLeft].translation.x,
             theRobotModel.limbs[MassCalibration::footLeft].translation.y,
             theRobotModel.limbs[MassCalibration::footLeft].translation.z - theRobotDimensions.heightLeg5Joint,
             3,
             ColorClasses::purple1);

    SPHERE3D("module:Walk2014Generator:nextStep", 0, 0, -theWalk2014GeneratorParams.walkHipHeight, 5, ColorClasses::blue);
    // Reference Drawings
    SPHERE3D("module:Walk2014Generator:nextStep",
             0,
             theRobotDimensions.yHipOffset,
             -theWalk2014GeneratorParams.walkHipHeight,
             2,
             ColorClasses::blue);
    SPHERE3D("module:Walk2014Generator:nextStep",
             0,
             -theRobotDimensions.yHipOffset,
             -theWalk2014GeneratorParams.walkHipHeight,
             2,
             ColorClasses::blue);
    LINE3D("module:Walk2014Generator:nextStep",
           0,
           theRobotDimensions.yHipOffset,
           -theWalk2014GeneratorParams.walkHipHeight,
           0,
           -theRobotDimensions.yHipOffset,
           -theWalk2014GeneratorParams.walkHipHeight,
           2,
           ColorClasses::blue);
  });
  // 9.2 Walk kicks
  // TODO [walk] integrate walk kicks
  /*
  if (getKickFootOffset)
    (generator.isLeftPhase ? leftFoot : rightFoot)
      .conc(getKickFootOffset(std::min(generator.t / generator.stepDuration, 1.f)));
  */

  // 9.3 Inverse kinematics
  VERIFY(
    InverseKinematic::calcLegJoints(leftFoot, rightFoot, {0, 0}, generator.jointRequest.jointAngles, theRobotDimensions) ||
    SystemCall::getMode() == SystemCall::logfileReplay);

  // 10. Set joint values and stiffness
  int stiffness = walkState == standing && theFrameInfo.getTimeSince(timeWhenStandBegan) > standStiffnessDelay
                    ? HardnessData::useDefault
                    : walkStiffness;
  for (uint8_t i = JointData::FirstLegJoint; i < JointData::numOfJoints; ++i) {
    generator.jointRequest.jointHardness.hardness[i] = stiffness;
  }

  // 10.1 Arms
  for (int joint = JointData::LShoulderPitch; joint < JointData::FirstLegJoint; ++joint) {
    generator.jointRequest.jointAngles.angles[joint] = 0_deg;
    generator.jointRequest.jointHardness.hardness[joint] = HardnessData::useDefault;
  }

  // 5.5 "natural" arm swing while walking to counterbalance foot swing
  generator.jointRequest.jointAngles.angles[JointData::LShoulderPitch] = (90_deg - forwardL * armShoulderPitchFactor);
  generator.jointRequest.jointAngles.angles[JointData::LShoulderRoll] =
    armShoulderRoll + std::abs(left) * armShoulderRollIncreaseFactor;
  generator.jointRequest.jointAngles.angles[JointData::LWristYaw] = -90_deg;
  generator.jointRequest.jointAngles.angles[JointData::RWristYaw] = 90_deg;
  generator.jointRequest.jointAngles.angles[JointData::RShoulderRoll] =
    -armShoulderRoll - std::abs(left) * armShoulderRollIncreaseFactor;

  generator.jointRequest.jointAngles.angles[JointData::RShoulderPitch] = (90_deg - forwardR * armShoulderPitchFactor);

  // if (theArmMotionRequest.motion[0] == ArmMotionRequest::ArmMotionId::back &&
  //     theArmMotionRequest.motion[1] == ArmMotionRequest::ArmMotionId::back) {
  //   forwardTiltOffset = fromDegrees(4.f);
  // } else if (theArmMotionRequest.motion[0] == ArmMotionRequest::ArmMotionId::back ||
  //            theArmMotionRequest.motion[1] == ArmMotionRequest::ArmMotionId::back) {
  //   forwardTiltOffset = fromDegrees(2.f);
  // } else {
  //   forwardTiltOffset = fromDegrees(0);
  // }
  // Compensate arm position's effect on COM by tilting torso
  compensateArmPosition(leftFoot, rightFoot, generator.jointRequest);

  // 7. Sagittal balance
  Angle balanceAdjustment =
    walkState == standing
      ? 0.f
      : filteredGyroY *
          (filteredGyroY > 0
             ? theWalk2014GeneratorParams.gyroForwardBalanceFactor
             : theWalk2014GeneratorParams.gyroBackwardBalanceFactor); // adjust ankle tilt in proportion to filtered gryoY
  generator.jointRequest.jointAngles.angles[generator.isLeftPhase ? JointData::RAnklePitch : JointData::LAnklePitch] +=
    balanceAdjustment;

  PLOT("module:Walk2014Generator:balanceAdjustmentPitch",
       toDegrees(balanceAdjustment) * 10); // adjusted to be visible on SimRobot plots
  PLOT("module:Walk2014Generator:imbalanceX", toDegrees(filteredGyroY) * 0.5);

  PLOT("module:Walk2014Generator:debug1", theWalk2014GeneratorParams.gyroForwardBalanceFactor * 1000);
  PLOT("module:Walk2014Generator:debug2", theWalk2014GeneratorParams.gyroBackwardBalanceFactor * 1000);

  // Lateral balance
  if (walkState == standing) {
    balanceAdjustment = filteredGyroX * theWalk2014GeneratorParams.gyroSidewaysBalanceFactor;
    generator.jointRequest.jointAngles.angles[JointData::LAnkleRoll] += balanceAdjustment;
    generator.jointRequest.jointAngles.angles[JointData::RAnkleRoll] += balanceAdjustment;
  }

  PLOT("module:Walk2014Generator:balanceAdjustmentRoll", toDegrees(balanceAdjustment) * 10);
  PLOT("module:Walk2014Generator:imbalanceY", toDegrees(filteredGyroX) * 0.5);

  // Head can move freely
  generator.jointRequest.jointAngles.angles[JointData::HeadPitch] =
    generator.jointRequest.jointAngles.angles[JointData::HeadYaw] = JointData::ignore;
#ifdef TARGET_SIM
  if (generator.t > generator.stepDuration) {
    generator.isLeftPhase = !generator.isLeftPhase;
    if (walkState != standing) {
      // 6.1 Recover previous "left" swing angle
      swingAngle = generator.isLeftPhase ? leftL : leftR;

      // 6.4 backup values
      forwardL0 = forwardL;
      forwardR0 = forwardR;
      turnRL0 = turnRL;

      // 6.2 Decide on timing of next walk step phase
      if (walkState != walking) {
        if (walkState != stopping ||
            (walkState == stopping && std::abs(turnRL) < thresholdStopStandTransition.rotation &&
             std::abs(forwardL * mmPerM) < thresholdStopStandTransition.translation.x &&
             std::abs(swingAngle) < thresholdStopStandTransition.translation.y && !theGroundContactState.contact) ||
            (walkState == stopping && theGroundContactState.contact)) {
          walkState = static_cast<WalkState>((walkState + 1) & 3);
          if (walkState == standing) {
            timeWhenStandBegan = theFrameInfo.time;
          }
        } else if (walkState == stopping) {
          weightShiftStatus = weightDidShift;
        }
      }
      // 6.3 reset step phase time
      generator.t = 0;
    }
  }
#endif

  // 6. Changing Support Foot. Note isLeftPhase means left foot is swing foot.
  // t>0.75*T tries to avoid bounce, especially when side-stepping
  // lastZMPL*ZMPL<0.0 indicates that support foot has changed
  // t>3*T tires to get out of "stuck" situations
  // a predicted foot support switch is used, if the current rotation speed is low

  const bool predictedSwitch = std::abs(generator.speed.rotation) < turnThresholdFootSupportPrediction &&
                               theFootSupport.predictedSwitched && useFootSupportSwitchPrediction;
  if ((generator.t > supportSwitchPhaseRangeUsed.min * generator.stepDuration && theFootSupport.switched) ||
      (predictedSwitch && generator.t > fastStartStepSupportSwitchPhaseRange.min * generator.stepDuration) ||
      (generator.t > supportSwitchPhaseRangeUsed.max * generator.stepDuration)) {
    // if the support switch is predicted, weightShiftStatus and isLeftPhase must be set differently
    const bool usePrediction = predictedSwitch && !theFootSupport.switched &&
                               (generator.t <= supportSwitchPhaseRangeUsed.max * generator.stepDuration);
    generator.usedPredictedSwitch = usePrediction;
    switchPhase = generator.t;
    maxFootHeight0 = maxFootHeight;
    weightShiftStatus = usePrediction
                          ? weightDidShift
                          : (generator.isLeftPhase != (theFootSupport.support < 0) ? weightDidShift : weightDidNotShift);
    generator.isLeftPhase = usePrediction ? !generator.isLeftPhase : theFootSupport.support < 0;

    footSoleDistanceAtStepStart = std::fabs(theRobotModel.soleLeft.translation.y - theRobotModel.soleRight.translation.y);

    if (walkState != standing) {
      // 6.1 Recover previous "left" swing angle
      swingAngle = generator.isLeftPhase ? leftL : leftR;

      // 6.4 backup values
      forwardL0 = forwardL;
      forwardR0 = forwardR;
      turnRL0 = turnRL;

      // 6.2 Decide on timing of next walk step phase
      if (walkState != walking) {
        if (walkState != stopping ||
            (walkState == stopping && std::abs(turnRL) < thresholdStopStandTransition.rotation &&
             std::abs(forwardL * mmPerM) < thresholdStopStandTransition.translation.x &&
             std::abs(swingAngle) < thresholdStopStandTransition.translation.y && !theGroundContactState.contact) ||
            (walkState == stopping && theGroundContactState.contact)) {
          walkState = static_cast<WalkState>((walkState + 1) & 3);
          if (walkState == standing) {
            timeWhenStandBegan = theFrameInfo.time;
          }
        } else if (walkState == stopping) {
          weightShiftStatus = weightDidShift;
        }
      }

      // 6.3 reset step phase time
      generator.t = 0;
    }
  } // end of changing support foot
}

void Walk2014Generator::calcNextStep(WalkGeneratorData& generator,
                                     const Pose2D& speed,
                                     const Pose2D& target,
                                     WalkGeneratorData::WalkMode walkMode) {
  Pose2D request = speed;
  Pose2D maxSpeed = this->theWalk2014GeneratorParams.maxSpeed;
  float maxSpeedBackwards = this->theWalk2014GeneratorParams.maxSpeedBackwards;
  float sideForwardMaxSpeed = this->theWalk2014GeneratorParams.sideForwardMaxSpeed;
  Pose2D returnOffset((generator.isLeftPhase ? -turnRL0 : turnRL0) * odometryScale.rotation,
                      (generator.isLeftPhase ? -forwardR0 : -forwardL0) * mmPerM * odometryScale.translation.x,
                      (theWalk2014GeneratorParams.walkHipHeight - theRobotDimensions.heightLeg5Joint) *
                        std::tan(swingAngle) * odometryScale.translation.y);
  emergencyLift = false;

  if (theFrameInfo.getTimeSince(timeWhenSlowWeightShiftsDetected) <= slowWaitShiftStandDelay) {
    request = Pose2D();
    walkMode = WalkGeneratorData::stepSizeMode;
  } else if (weightShiftStatus == emergencyStep) {
    request = Pose2D(0_deg, 0, generator.isLeftPhase ? emergencyStepSize : -emergencyStepSize);
    walkMode = WalkGeneratorData::stepSizeMode;
    weightShiftStatus = weightDidShift;
    emergencyLift = true;
  }

  if (walkMode == WalkGeneratorData::targetMode) {
    ASSERT(speed.rotation > 0.f && speed.translation.x > 0.f && speed.translation.y > 0.f);
    maxSpeed = Pose2D(
      std::min(speed.rotation / odometryScale.rotation,
               static_cast<float>(this->theWalk2014GeneratorParams.maxSpeed.rotation)),
      std::min(speed.translation.x / odometryScale.translation.x, this->theWalk2014GeneratorParams.maxSpeed.translation.x),
      std::min(speed.translation.y / odometryScale.translation.y, this->theWalk2014GeneratorParams.maxSpeed.translation.y));
    sideForwardMaxSpeed = std::min(speed.translation.x / odometryScale.translation.x /
                                     this->theWalk2014GeneratorParams.maxSpeed.translation.x * sideForwardMaxSpeed,
                                   this->theWalk2014GeneratorParams.sideForwardMaxSpeed);
    maxSpeedBackwards =
      std::min(speed.translation.x / odometryScale.translation.x, this->theWalk2014GeneratorParams.maxSpeedBackwards);
    // Remove the offset that will be covered just by returning the swing leg
    forward = (target.translation.x - returnOffset.translation.x) / odometryScale.translation.x;
    left = (target.translation.y - returnOffset.translation.y) / odometryScale.translation.y;
    // Hack to make robot faster
    // TODO: figure out how to make this work
    // if (std::fabs(left) > triggerForwardBoostByThisLeftAmount)
    // maxSpeed.translation.x = sideForwardMaxSpeed;

    // If the leg swings in target direction, consider that the next step will move the robot the same distance again
    if (left * (generator.isLeftPhase ? 1.f : -1.f) > 0) {
      left *= 0.5f;
    }
    turn = (target.rotation - returnOffset.rotation) / odometryScale.rotation;
    generator.stepDuration =
      (theWalk2014GeneratorParams.baseWalkPeriod +
       theWalk2014GeneratorParams.forwardsWalkPeriodIncreaseFactor * forward / generator.maxSpeed.translation.x +
       theWalk2014GeneratorParams.sidewaysWalkPeriodIncreaseFactor * std::abs(left)) /
      1000.f;
    generator.speed = Pose2D(turn / generator.stepDuration, forward / generator.stepDuration, left / generator.stepDuration);
    if (ellipsoidClampWalk(maxSpeed,
                           maxSpeedBackwards,
                           generator.speed.translation.x,
                           generator.speed.translation.y,
                           generator.speed.rotation)) {
      walkMode = WalkGeneratorData::speedMode;
      request = Pose2D(target.rotation * targetModeSpeedFactor,
                       target.translation * targetModeSpeedFactor); // targetModeSpeedFactor = 2, why we double the speed ?
    } else {
      // Consider in the speed that half of the step is returning to origin
      float turnFactor = turn * (generator.isLeftPhase ? 1.f : -1.f) > 0 ? 1.f - insideTurnRatio : insideTurnRatio;
      float leftFactor = left * (generator.isLeftPhase ? 1.f : -1.f) > 0 ? 1.f : 0.f;
      generator.speed = Pose2D(
        turnFactor * generator.speed.rotation * odometryScale.rotation + returnOffset.rotation / generator.stepDuration,
        0.5f * generator.speed.translation.x * odometryScale.translation.x +
          returnOffset.translation.x / generator.stepDuration,
        leftFactor * left * odometryScale.translation.y + returnOffset.translation.y / generator.stepDuration);
      generator.upcomingOdometryOffset =
        Pose2D(turnFactor * generator.speed.rotation * generator.stepDuration * odometryScale.rotation +
               returnOffset.rotation) +
        Pose2D(0.5f * generator.speed.translation.x * generator.stepDuration * odometryScale.translation.x +
                 returnOffset.translation.x,
               leftFactor * left * generator.stepDuration * odometryScale.translation.y + returnOffset.translation.y) +
        Pose2D(turnFactor * generator.speed.rotation * generator.stepDuration * odometryScale.rotation) +
        Pose2D(0.5f * generator.speed.translation.x * generator.stepDuration * odometryScale.translation.x,
               leftFactor * left * generator.stepDuration * odometryScale.translation.y);
    }
  }

  if (walkMode == WalkGeneratorData::speedMode) {
    forward = request.translation.x / odometryScale.translation.x;
    left = request.translation.y / odometryScale.translation.y;
    turn = request.rotation / odometryScale.rotation;

    // Hack to make robot faster
    // TODO: figure out how to make this work
    // If originally in targetMode, sideForwardMaxSpeed is already clipped based on the request speed
    // if (std::fabs(left) > triggerForwardBoostByThisLeftAmount)
    // maxSpeed.translation.x = sideForwardMaxSpeed;
    // Scale back values to try to ensure stability.
    ellipsoidClampWalk(maxSpeed, maxSpeedBackwards, forward, left, turn);

    // If switching direction, first stop if new speed is not reachable through acceleration
    if (lastForward * forward < 0.f && std::abs(lastForward) > std::abs(maxAcceleration.x)) {
      forward = 0.f;
    }

    // Limit acceleration and deceleration of forward movement
    if (lastForward > 0.f || (lastForward == 0.f && forward > 0.f)) {
      forward = lastForward + Rangef(-maxDeceleration.x, maxAcceleration.x).limit(forward - lastForward);
    } else {
      forward = lastForward + Rangef(-maxAcceleration.x, maxDeceleration.x).limit(forward - lastForward);
    }

    // If switching direction, first stop if new speed is not reachable through acceleration
    if (lastLeft * left < 0.f && std::abs(lastLeft) > std::abs(maxAcceleration.y)) {
      left = 0.f;
    }

    // Limit acceleration and deceleration of sideways movement
    if (lastLeft > 0.f || (lastLeft == 0.f && left > 0.f)) {
      left = lastLeft + Rangef(-maxDeceleration.y, maxAcceleration.y).limit(left - lastLeft);
    } else {
      left = lastLeft + Rangef(-maxAcceleration.y, maxDeceleration.y).limit(left - lastLeft);
    }

    generator.stepDuration =
      (theWalk2014GeneratorParams.baseWalkPeriod +
       theWalk2014GeneratorParams.forwardsWalkPeriodIncreaseFactor * forward / generator.maxSpeed.translation.x +
       theWalk2014GeneratorParams.sidewaysWalkPeriodIncreaseFactor * std::abs(left)) /
      1000.f;

    // Consider in the speed that half of the step is returning to origin
    float turnFactor = turn * (generator.isLeftPhase ? 1.f : -1.f) > 0 ? 1.f - insideTurnRatio : insideTurnRatio;
    float leftFactor = left * (generator.isLeftPhase ? 1.f : -1.f) > 0 ? 1.f : 0.f;
    generator.speed =
      Pose2D(turnFactor * turn * odometryScale.rotation + returnOffset.rotation / generator.stepDuration,
             0.5f * forward * odometryScale.translation.x + returnOffset.translation.x / generator.stepDuration,
             leftFactor * left * odometryScale.translation.y + returnOffset.translation.y / generator.stepDuration);
    generator.upcomingOdometryOffset =
      Pose2D(turnFactor * turn * generator.stepDuration * odometryScale.rotation + returnOffset.rotation) +
      Pose2D(0.5f * forward * generator.stepDuration * odometryScale.translation.x + returnOffset.translation.x,
             leftFactor * left * generator.stepDuration * odometryScale.translation.y + returnOffset.translation.y) +
      Pose2D(turnFactor * turn * generator.stepDuration * odometryScale.rotation) +
      Pose2D(0.5f * forward * generator.stepDuration * odometryScale.translation.x,
             leftFactor * left * generator.stepDuration * odometryScale.translation.y);
  } else if (walkMode == WalkGeneratorData::stepSizeMode) {
    forward = request.translation.x;
    left = request.translation.y;
    turn = request.rotation;
    generator.stepDuration =
      (theWalk2014GeneratorParams.baseWalkPeriod +
       theWalk2014GeneratorParams.forwardsWalkPeriodIncreaseFactor * forward / generator.maxSpeed.translation.x +
       theWalk2014GeneratorParams.sidewaysWalkPeriodIncreaseFactor * std::abs(left)) /
      1000.f;

    // Consider in the speed that half of the step is returning to origin
    float turnFactor = turn * (generator.isLeftPhase ? 1.f : -1.f) > 0 ? 1.f - insideTurnRatio : insideTurnRatio;
    float leftFactor = left * (generator.isLeftPhase ? 1.f : -1.f) > 0 ? 1.f : 0.f;
    generator.speed =
      Pose2D((turnFactor * turn * odometryScale.rotation + returnOffset.rotation) / generator.stepDuration,
             (0.5f * forward * odometryScale.translation.x + returnOffset.translation.x) / generator.stepDuration,
             (leftFactor * left * odometryScale.translation.y + returnOffset.translation.y) / generator.stepDuration);
    generator.upcomingOdometryOffset =
      Pose2D(turnFactor * turn * odometryScale.rotation + returnOffset.rotation) +
      Pose2D(0.5f * forward * odometryScale.translation.x + returnOffset.translation.x,
             leftFactor * left * odometryScale.translation.y + returnOffset.translation.y) +
      Pose2D(turnFactor * turn * odometryScale.rotation) +
      Pose2D(0.5f * forward * odometryScale.translation.x, leftFactor * left * odometryScale.translation.y);
  }

  if (walkMode == WalkGeneratorData::speedMode) {
    lastForward = forward;
    lastLeft = left;

    // 1.6 Walk Calibration
    // The definition of forward, left and turn is the actual distance/angle traveled in one second.
    // It is scaled down to the duration of a single step.
    forward *= generator.stepDuration;
    left *= generator.stepDuration;
    turn *= generator.stepDuration;
  } else {
    lastForward = forward / generator.stepDuration;
    lastLeft = left / generator.stepDuration;
  }

  forward /= mmPerM; // in m/s
  left /= mmPerM;    // in m/s

  // 5.1 Calculate the height to lift each swing foot
  maxFootHeight =
    theWalk2014GeneratorParams.baseFootLift / mmPerM +
    std::abs(forward) * (forward < 0 ? footLiftIncreaseFactorBackwards
                                     : (footLiftIncreaseFactorForwards[generator.isLeftPhase ? Leg::left : Leg::right])) +
    std::abs(left) * footLiftIncreaseFactorSidewards;
  if (emergencyLift) {
    maxFootHeight *= emergencyStepHeightFactor;
  }

  generator.turn = static_cast<float>(turn);
  generator.forward = forward;
  generator.left = left;
}

void Walk2014Generator::calcFootOffsets(WalkGeneratorData& generator,
                                        float swingFootSign,
                                        float forwardSwing0,
                                        float forwardSupport0,
                                        float& forwardSwing,
                                        float& forwardSupport,
                                        Angle& leftSwing,
                                        Angle& leftSupport,
                                        float& footHeightSwing,
                                        float& footHeightSupport) {
  if (weightShiftStatus == weightDidShift) {
    // 5.3.1 forward: steps from (previous) -forward/2 to +forward/2, i.e. the target is forward/2
    forwardSupport = forwardSupport0 +
                     (forward / 2.f - forwardSupport0) * Rangef::ZeroOneRange().limit(generator.t / generator.stepDuration);
    forwardSwing = forwardSwing0 + (-forward / 2.f - forwardSwing0) *
                                     parabolicStep(generator.t, generator.stepDuration); // swing-foot follow-through

    // 5.3.4 left: steps from left0 to +left in one step and from (previous) -left to 0 in the next
    float legLength = (theWalk2014GeneratorParams.walkHipHeight - theRobotDimensions.heightLeg5Joint) / mmPerM;
    float left0 = std::tan(-swingAngle) * legLength;
    leftSupport = std::atan2(left0 + ((left * swingFootSign > 0 ? left : 0.f) - left0) *
                                       Rangef::ZeroOneRange().limit(generator.t / generator.stepDuration),
                             legLength);
    leftSwing = -std::atan2(left0 + ((left * swingFootSign > 0 ? left : 0.f) - left0) *
                                      parabolicStep(generator.t, generator.stepDuration),
                            legLength);

    // 5.3.5 turn
    turnRL =
      turnRL0 + ((turn * swingFootSign > 0 ? 1.f - insideTurnRatio : insideTurnRatio) * swingFootSign * turn - turnRL0) *
                  Rangef::ZeroOneRange().limit(generator.t / generator.stepDuration);
  }

  // 5.3.6 determine how high to lift the swing foot off the ground
  footHeightSwing = maxFootHeight * parabolicReturn(generator.t / generator.stepDuration); // lift swing foot
  footHeightSupport =
    maxFootHeight0 *
    parabolicReturn((switchPhase + generator.t) / generator.stepDuration); // return support foot to 0 if it was still lifted
}

Pose2D Walk2014Generator::calcOdometryOffset(WalkGeneratorData& generator, bool isLeftSwingFoot) {
  // When the support foot switches, the old swing foot still moves forward for a few frames
  // (because it was commanded to do so because of the 3-4 frame delay), which causes the foot
  // to push the robot backwards. Other solution would be to save the last 4 commanded odometry
  // offsets and add the negative sum of them once at the start of the next walk phase.
  if (generator.t <= 3 * Constants::motionCycleTime && weightShiftStatus == weightDidShift) {
    return Pose2D();
  }
  // Work out incremental forward, left, and turn values for next time step
  Pose2D offset((turnRL - prevTurn) * (isLeftSwingFoot ? 1.f : -1.f) * odometryScale.rotation,
                (isLeftSwingFoot ? forwardR - prevForwardR : forwardL - prevForwardL) * mmPerM * odometryScale.translation.x,
                (theWalk2014GeneratorParams.walkHipHeight - theRobotDimensions.heightLeg5Joint) *
                  (isLeftSwingFoot ? std::tan(leftR) - std::tan(prevLeftR) : std::tan(leftL) - std::tan(prevLeftL)) *
                  odometryScale.translation.y);

  // backup values for next computation
  prevTurn = turnRL;
  prevLeftL = leftL;
  prevLeftR = leftR;
  prevForwardL = forwardL;
  prevForwardR = forwardR;

  return offset;
}

float Walk2014Generator::calcWalkVolume(float forward, float left, float turn) const {
  return std::pow(std::pow(forward, walkVolumeTranslationExponent) + std::pow(left, walkVolumeTranslationExponent),
                  (walkVolumeRotationExponent / walkVolumeTranslationExponent)) +
         std::pow(turn, walkVolumeRotationExponent);
}

bool Walk2014Generator::ellipsoidClampWalk(
  const Pose2D& maxSpeed, float maxSpeedBackwards, float& forward, float& left, float& turn) const {
  // Values in range [-1..1]
  float forwardAmount = forward / (forward >= 0.f ? maxSpeed.translation.x : maxSpeedBackwards);
  float leftAmount = left / maxSpeed.translation.y;
  float turnAmount = turn / maxSpeed.rotation;

  // Divide the speeds by a factor in [0,1] to force the ellipsoidal clamping
  forwardAmount /= theWalk2014GeneratorParams.maxSpeedDivider;
  leftAmount /= theWalk2014GeneratorParams.maxSpeedDivider;
  turnAmount /= theWalk2014GeneratorParams.maxSpeedDivider;

  float factor = std::max(std::max(std::abs(forwardAmount), std::abs(leftAmount)), std::abs(turnAmount));
  bool clamp = factor > 1.f;
  if (clamp) {
    forwardAmount /= factor;
    leftAmount /= factor;
  }
  // Clip based on a lower max turn speed. This causes a stronger clipping for forward and left, but allows for fast turns.
  // Otherwise the robot might fall more often.
  // float clampTurnAmount = std::min(std::abs(turn) / useMaxTurnSpeedForClampWalk, 1.f);

  float clampTurnAmount = 0;
  if (turn >= 0) {
    clampTurnAmount = std::min(turn / maxSpeed.rotation, 1.f); // useMaxTurnSpeedForClampWalk
  } else {
    clampTurnAmount = std::max(turn / maxSpeed.rotation, -1.f);
  }

  // see if the point we are given is already inside the allowed walk params volume
  if (calcWalkVolume(std::abs(forwardAmount), std::abs(leftAmount), std::abs(clampTurnAmount)) > 1.f) {
    clamp = true;
    float scale = 0.5f;
    float high = 1.f;
    float low = 0.f;

    // This is basically a binary search to find the point on the surface.
    for (unsigned i = 0; i < 10; ++i) {
      // give priority to turn. keep it the same
      if (calcWalkVolume(std::abs(forwardAmount) * scale, std::abs(leftAmount) * scale, std::abs(clampTurnAmount)) > 1.f) {
        high = scale;
      } else {
        low = scale;
      }
      scale = (low + high) / 2.f;
    }

    forwardAmount *= scale;
    leftAmount *= scale;
  }
  forward = (forward >= 0.f ? maxSpeed.translation.x : maxSpeedBackwards) * forwardAmount *
            theWalk2014GeneratorParams.maxSpeedDivider;
  left = maxSpeed.translation.y * leftAmount * theWalk2014GeneratorParams.maxSpeedDivider;
  turn = maxSpeed.rotation * clampTurnAmount * theWalk2014GeneratorParams.maxSpeedDivider;

  return clamp;
}

float Walk2014Generator::parabolicReturn(float f) const {
  Rangef::ZeroOneRange().clamp(f);

  if (f < 0.25f) {
    return 8.f * f * f;
  } else if (f < 0.75f) {
    float x = f - 0.5f;
    return 1.f - 8.f * x * x;
  } else {
    float x = 1.f - f;
    return 8.f * x * x;
  }
}

float Walk2014Generator::parabolicStep(float time, float period) const {
  float timeFraction = Rangef::ZeroOneRange().limit(time / period);
  if (timeFraction < 0.5f) {
    return 2.f * timeFraction * timeFraction;
  } else {
    return 4.f * timeFraction - 2.f * timeFraction * timeFraction - 1.f;
  }
}

void Walk2014Generator::compensateArmPosition(const Pose3D& leftFoot, const Pose3D& rightFoot, JointRequest& jointRequest) {

  JointRequest temp = jointRequest;
  for (int joint = 0; joint < JointData::FirstArmJoint; ++joint) {
    temp.jointAngles.angles[joint] = theJointRequest.jointAngles.angles[joint] != JointData::off
                                       ? theJointRequest.jointAngles.angles[joint]
                                       : theJointData.angles[joint];
  }
  RobotModel withWalkGeneratorArms(temp.jointAngles, theRobotDimensions, theMassCalibration);

  for (int joint = JointData::FirstArmJoint; joint < JointData::FirstLegJoint; ++joint) {
    temp.jointAngles.angles[joint] = theJointRequest.jointAngles.angles[joint] != JointData::off
                                       ? theJointRequest.jointAngles.angles[joint]
                                       : theJointData.angles[joint];
  }
  RobotModel balanced(temp.jointAngles, theRobotDimensions, theMassCalibration);

  for (int i = 0; i < numOfComIterations; ++i) {
    Quaternionf torsoRotation = Rotation::aroundY(-torsoTilt);
    VERIFY(InverseKinematic::calcLegJoints(leftFoot, rightFoot, torsoRotation, temp.jointAngles, theRobotDimensions) ||
           SystemCall::getMode() == SystemCall::logfileReplay);
    ForwardKinematic::calculateLegChain(Leg::left, temp.jointAngles, theRobotDimensions, balanced.limbs);
    ForwardKinematic::calculateLegChain(Leg::right, temp.jointAngles, theRobotDimensions, balanced.limbs);
    balanced.updateCenterOfMass(theMassCalibration);
    Eigen::Vector3f CoM(balanced.centerOfMass.x, balanced.centerOfMass.y, balanced.centerOfMass.z);
    Eigen::Vector3f balancedCom = torsoRotation * CoM;
    torsoTilt += (balancedCom.x() - withWalkGeneratorArms.centerOfMass.x) * pComFactor;
  }
  float tiltIncrease =
    mmPerM * (forward >= 0 ? forward * comTiltForwardIncreaseFactor : forward * comTiltBackwardIncreaseFactor) +
    forwardTiltOffset;
  VERIFY(
    InverseKinematic::calcLegJoints(
      leftFoot, rightFoot, {0.f, tiltIncrease - torsoTilt * comTiltFactor}, jointRequest.jointAngles, theRobotDimensions) ||
    SystemCall::getMode() == SystemCall::logfileReplay);
}
