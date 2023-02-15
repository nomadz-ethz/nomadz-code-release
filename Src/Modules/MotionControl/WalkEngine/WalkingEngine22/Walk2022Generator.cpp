/**
 * @file Walk2022Generator.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <algorithm>
#include "Core/Math/Rotation.h"
#include "Core/Range.h"
#include "Core/System/BHAssert.h"
#include "Core/System/SystemCall.h"
#include "Core/Math/Constants.h"
#include "Core/Streams/InStreams.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Debugging/DebugDrawings3D.h"

#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Motion/ForwardKinematic.h"

#include "Representations/Sensing/RobotModel.h"

#include "Walk2022Generator.h"

MAKE_MODULE(Walk2022Generator, Motion Control);

static const float mmPerM = 1000.f;

Walk2022Generator::Walk2022Generator() {
  InMapFile stream("walk2022GeneratorCommon.cfg");
  if (stream.exists()) {
    stream >> static_cast<Walk2022GeneratorCommon&>(*this);
  }
}

void Walk2022Generator::update(WalkGeneratorData& generator) {
  DECLARE_DEBUG_DRAWING3D("module:Walk2022Generator:nextStep", "robot");

  MODIFY("parameters:Walk2022Generator:params", theWalk2022GeneratorParams);
  MODIFY("parameters:Walk2022Generator:common", static_cast<Walk2022GeneratorCommon&>(*this));

  // Use other parameters in demo games to take care of the robots

  generator.reset = [this, &generator]() { reset(generator); };

  generator.calcJoints = [this,
                          &generator](const Pose2D& speed, const Pose2D& target, WalkGeneratorData::WalkMode walkMode) {
    calcJoints(generator, speed, target, walkMode);
  };

  generator.maxSpeed = theWalk2022GeneratorParams.maxSpeed.elementwiseMul(odometryScale);

  filteredGyroX = theWalk2022GeneratorParams.gyroLowPassRatio * filteredGyroX +
                  (1.f - theWalk2022GeneratorParams.gyroLowPassRatio) * theInertiaSensorData.gyro.x;
  filteredGyroY = theWalk2022GeneratorParams.gyroLowPassRatio * filteredGyroY +
                  (1.f - theWalk2022GeneratorParams.gyroLowPassRatio) * theInertiaSensorData.gyro.y;
}

void Walk2022Generator::reset(WalkGeneratorData& generator) {
  generator.stepDuration = 0.f;
  generator.leftRefPoint = Vector3<>(
    -theWalk2022GeneratorParams.torsoOffset, theRobotDimensions.yHipOffset, -theWalk2022GeneratorParams.walkHipHeight);
  generator.rightRefPoint = Vector3<>(
    -theWalk2022GeneratorParams.torsoOffset, -theRobotDimensions.yHipOffset, -theWalk2022GeneratorParams.walkHipHeight);
  generator.t = 0.f;
  generator.speed = Pose2D();
  generator.upcomingOdometryOffset = Pose2D();
  generator.walkState = WalkGeneratorData::standing;
  timeWhenStandBegan = theFrameInfo.time;
  if (theRobotInfo.penalty != PENALTY_NONE && theRobotInfo.penalty != PENALTY_SPL_ILLEGAL_MOTION_IN_SET) {
    filteredGyroX = filteredGyroY = 0;
  }

  currentSpeed = Pose2D();
  generator.leftFootOffset0 = leftFootOffset = Vector3<>();
  generator.rightFootOffset0 = leftFootOffset = Vector3<>();
  generator.turnRL0 = 0;
  generator.switchPhase = 0.f;
  generator.weightShiftStatus = WalkGeneratorData::weightDidNotShift;

  generator.swingControlPoint = generator.swingControlPoint0 = Vector3<>();
  prevLeftFootOffset = prevRightFootOffset = Vector3<>();
  prevTurn = 0_deg;
  torsoTilt = 0_deg;
  footSoleDistanceAtStepStart = std::fabs(theRobotModel.soleLeft.translation.y - theRobotModel.soleRight.translation.y);
}

void Walk2022Generator::calcJoints(WalkGeneratorData& generator,
                                   const Pose2D& speed,
                                   const Pose2D& target,
                                   WalkGeneratorData::WalkMode walkMode) {

  // 1. Read in new walk values (forward, left, turn, power) only at the start of a walk step cycle, ie when t = 0
  if (generator.t == 0) {
    Pose2D returnOffset =
      Pose2D((generator.isLeftPhase ? -generator.turnRL0 : generator.turnRL0),
             (generator.isLeftPhase ? -generator.rightFootOffset0.x : -generator.leftFootOffset0.x) * mmPerM,
             generator.leftFootOffset0.y * mmPerM)
        .elementwiseMul(odometryScale);
    generator.returnOffset = returnOffset;
    currentSpeed = thePlannedSteps.currentSpeed;
    generator.stepDuration = thePlannedSteps.stepDuration;

    generator.swingControlPoint =
      Vector3<>(0.0, 0.0, thePlannedSteps.maxFootHeight); // + Vector3<>(currentSpeed.translation) / 10;
    if (generator.isLeftPhase) {
      leftTraj.clear();
      rightTraj.clear();
    }
  }

  // 2. Update timer
  if (generator.t > generator.stepDuration - 1.1 * Constants::motionCycleTime &&
      theFallDownState.state != FallDownState::upright) {
    generator.t += 0;
    if (generator.stepDuration > theWalk2022GeneratorParams.baseWalkPeriod - Constants::motionCycleTime) {
      generator.t -= 0.5 * Constants::motionCycleTime;
      generator.stepDuration -= 0.5 * Constants::motionCycleTime;
    }
  } else {
    generator.t += Constants::motionCycleTime;
  }

  // 3. Determine Walk2014 Option
  if (generator.walkState != WalkGeneratorData::standing && currentSpeed == Pose2D()) {
    generator.walkState = WalkGeneratorData::stopping;
  } else if (generator.walkState != WalkGeneratorData::walking && currentSpeed != Pose2D()) {
    generator.walkState = WalkGeneratorData::starting;
  }

  // 5. Determine walk variables throughout the walk step phase

  Rangef supportSwitchPhaseRangeUsed(supportSwitchPhaseRange.min, supportSwitchPhaseRange.max);
  if (generator.walkState == WalkGeneratorData::standing) {
    footSoleDistanceAtStepStart = std::fabs(theRobotModel.soleLeft.translation.y - theRobotModel.soleRight.translation.y);
    generator.stepDuration = generator.t = 0.f;
    leftFootOffset.z = rightFootOffset.z = 0; // don't move on starting
  } else {
    // 5.3 Calculate intra-walkphase forward, left and turn at time-step dt
    generateStepTrajectoryBezier(
      generator, thePlannedSteps.nextLeftStep, thePlannedSteps.nextRightStep, leftFootOffset, rightFootOffset, turnRL);

    // 5.4 Special conditions when priming the walk
    // forward check is not needed, because forward is clipped with the maxAcceleration
    // check is used to decide, if the robot can execute a real step as a starting step
    if (generator.walkState == WalkGeneratorData::starting) {
      if ((std::abs(currentSpeed.rotation) > fastStartStepMaxTurn ||
           footSoleDistanceAtStepStart > fastStartStepMaxSoleDistance)) {

        leftFootOffset.z *= footLiftFirstStepFactor; // reduce max lift due to short duration
        rightFootOffset.z *= footLiftFirstStepFactor;
        leftFootOffset.x = leftFootOffset.y = 0; // don't move on starting
        rightFootOffset.x = rightFootOffset.y = 0;
        generator.turnRL0 = turnRL = 0;
        generator.speed = Pose2D();
      } else {
        supportSwitchPhaseRangeUsed = fastStartStepSupportSwitchPhaseRange;
      }
      generator.weightShiftStatus = WalkGeneratorData::weightDidShift;
      if (currentSpeed.translation.y != 0.f) { // make first real step in direction of movement
        generator.isLeftPhase = currentSpeed.translation.y > 0;
      }
    }
  }
  // 8. Odometry update for localization
  generator.odometryOffset = calcOdometryOffset(generator, generator.isLeftPhase);

  // 9.1 Foot poses
  leftFoot = Pose3D(generator.leftRefPoint).translate(leftFootOffset * mmPerM).rotateZ(turnRL);

  rightFoot = Pose3D(generator.rightRefPoint).translate(rightFootOffset * mmPerM).rotateZ(-turnRL);

  rightTraj.push_back(std::make_pair(-turnRL, rightFoot.translation));
  leftTraj.push_back(std::make_pair(turnRL, leftFoot.translation));

  COMPLEX_DRAWING3D("module:Walk2022Generator:nextStep", {
    // Requested Trajectory
    int colorAlphaIndex = 0;
    for (auto& pair : rightTraj) {
      SPHERE3D_VEC("module:Walk2022Generator:nextStep",
                   pair.second,
                   2,
                   ColorRGBA(0, 255, 0, (int)(255 * colorAlphaIndex / rightTraj.size())));
      Vector3<> direction = pair.second + Vector3<>(std::cos(pair.first), std::sin(pair.first), 0.0) * 50;
      SPHERE3D_VEC("module:Walk2022Generator:nextStep",
                   direction,
                   2,
                   ColorRGBA(0, 255, 0, (int)(255 * colorAlphaIndex / leftTraj.size())));
      colorAlphaIndex++;
    }
    colorAlphaIndex = 0;
    for (auto& pair : leftTraj) {
      SPHERE3D_VEC("module:Walk2022Generator:nextStep",
                   pair.second,
                   2,
                   ColorRGBA(0, 255, 0, (int)(255 * colorAlphaIndex / leftTraj.size())));
      Vector3<> direction = pair.second + Vector3<>(std::cos(pair.first), std::sin(pair.first), 0.0) * 50;
      SPHERE3D_VEC("module:Walk2022Generator:nextStep",
                   direction,
                   2,
                   ColorRGBA(0, 255, 0, (int)(255 * colorAlphaIndex / leftTraj.size())));
      colorAlphaIndex++;
    }
    SPHERE3D_VEC("module:Walk2022Generator:nextStep", rightFoot.translation, 3, ColorClasses::red);
    SPHERE3D_VEC("module:Walk2022Generator:nextStep", leftFoot.translation, 3, ColorClasses::red);

    SPHERE3D("module:Walk2022Generator:nextStep", 0, 0, -theWalk2022GeneratorParams.walkHipHeight, 5, ColorClasses::blue);
    // Reference Drawings
    SPHERE3D("module:Walk2022Generator:nextStep",
             0,
             theRobotDimensions.yHipOffset,
             -theWalk2022GeneratorParams.walkHipHeight,
             2,
             ColorClasses::blue);
    SPHERE3D("module:Walk2022Generator:nextStep",
             0,
             -theRobotDimensions.yHipOffset,
             -theWalk2022GeneratorParams.walkHipHeight,
             2,
             ColorClasses::blue);
    LINE3D("module:Walk2022Generator:nextStep",
           0,
           theRobotDimensions.yHipOffset,
           -theWalk2022GeneratorParams.walkHipHeight,
           0,
           -theRobotDimensions.yHipOffset,
           -theWalk2022GeneratorParams.walkHipHeight,
           2,
           ColorClasses::blue);
  });
  // 9.2 Walk kicks

  // 9.3 Inverse kinematics
  VERIFY(
    InverseKinematic::calcLegJoints(leftFoot, rightFoot, {0, 0}, generator.jointRequest.jointAngles, theRobotDimensions) ||
    SystemCall::getMode() == SystemCall::logfileReplay);

  // 10. Set joint values and stiffness
  int stiffness =
    generator.walkState == WalkGeneratorData::standing && theFrameInfo.getTimeSince(timeWhenStandBegan) > standStiffnessDelay
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
  generator.jointRequest.jointAngles.angles[JointData::LShoulderPitch] =
    (90_deg + leftFootOffset.x * armShoulderPitchFactor);
  generator.jointRequest.jointAngles.angles[JointData::LShoulderRoll] =
    armShoulderRoll + std::abs(currentSpeed.translation.y) * armShoulderRollIncreaseFactor;
  generator.jointRequest.jointAngles.angles[JointData::LWristYaw] = -90_deg;

  generator.jointRequest.jointAngles.angles[JointData::RWristYaw] = 90_deg;
  generator.jointRequest.jointAngles.angles[JointData::RShoulderRoll] =
    -armShoulderRoll - std::abs(currentSpeed.translation.y) * armShoulderRollIncreaseFactor;

  generator.jointRequest.jointAngles.angles[JointData::RShoulderPitch] =
    (90_deg + rightFootOffset.x * armShoulderPitchFactor);

  // Compensate arm position's effect on COM by tilting torso
  compensateArmPosition(leftFoot, rightFoot, generator.jointRequest);

  // 7. Sagittal balance
  Angle balanceAdjustment =
    generator.walkState == WalkGeneratorData::standing
      ? 0.f
      : filteredGyroY *
          (filteredGyroY > 0
             ? theWalk2022GeneratorParams.gyroForwardBalanceFactor
             : theWalk2022GeneratorParams.gyroBackwardBalanceFactor); // adjust ankle tilt in proportion to filtered gryoY
  generator.jointRequest.jointAngles.angles[generator.isLeftPhase ? JointData::RAnklePitch : JointData::LAnklePitch] +=
    balanceAdjustment;

  // Lateral balance
  if (generator.walkState == WalkGeneratorData::standing) {
    balanceAdjustment = filteredGyroX * theWalk2022GeneratorParams.gyroSidewaysBalanceFactor;
    generator.jointRequest.jointAngles.angles[JointData::LAnkleRoll] += balanceAdjustment;
    generator.jointRequest.jointAngles.angles[JointData::RAnkleRoll] += balanceAdjustment;
  }

  // Head can move freely
  generator.jointRequest.jointAngles.angles[JointData::HeadPitch] =
    generator.jointRequest.jointAngles.angles[JointData::HeadYaw] = JointData::ignore;

  if (generator.t > generator.stepDuration) {
    generator.isLeftPhase = !generator.isLeftPhase;
    stateCheck(generator, leftFootOffset, rightFootOffset, turnRL);
  }

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
    generator.switchPhase = generator.t;
    generator.weightShiftStatus =
      usePrediction ? WalkGeneratorData::weightDidShift
                    : (generator.isLeftPhase != (theFootSupport.support < 0) ? WalkGeneratorData::weightDidShift
                                                                             : WalkGeneratorData::weightDidNotShift);
    generator.isLeftPhase = usePrediction ? !generator.isLeftPhase : theFootSupport.support < 0;

    footSoleDistanceAtStepStart = std::fabs(theRobotModel.soleLeft.translation.y - theRobotModel.soleRight.translation.y);

    stateCheck(generator, leftFootOffset, rightFootOffset, turnRL);

  } // end of changing support foot
}

void Walk2022Generator::stateCheck(WalkGeneratorData& generator,
                                   Vector3<> leftFootOffset,
                                   Vector3<> rightFootOffset,
                                   float turnRL) {
  if (generator.walkState != WalkGeneratorData::standing) {
    generator.swingControlPoint0 = generator.swingControlPoint;
    generator.leftFootOffset0 = leftFootOffset;
    generator.rightFootOffset0 = rightFootOffset;
    generator.turnRL0 = turnRL;
    generator.t = 0;
  }
  switch (generator.walkState) {
  case WalkGeneratorData::standing:
    /* if (currentSpeed != Pose2D()) {
      generator.walkState = WalkGeneratorData::starting;
    } */
    break;

  case WalkGeneratorData::starting:
    generator.walkState = WalkGeneratorData::walking;
    break;

  case WalkGeneratorData::walking:
    /*     if (currentSpeed == Pose2D()) {
          generator.walkState = WalkGeneratorData::stopping;
        } */
    break;

  case WalkGeneratorData::stopping:
    if ((std::abs(turnRL) < thresholdStopStandTransition.rotation &&
         std::abs(leftFootOffset.x * mmPerM) < thresholdStopStandTransition.translation.x) ||
        theGroundContactState.contact) {
      generator.walkState = WalkGeneratorData::standing;
      timeWhenStandBegan = theFrameInfo.time;
    } else {
      generator.weightShiftStatus = WalkGeneratorData::weightDidShift;
    }
    break;

  default:
    break;
  }
}

Pose2D Walk2022Generator::calcOdometryOffset(WalkGeneratorData& generator, bool isLeftSwingFoot) {
  // When the support foot switches, the old swing foot still moves forward for a few frames
  // (because it was commanded to do so because of the 3-4 frame delay), which causes the foot
  // to push the robot backwards. Other solution would be to save the last 4 commanded odometry
  // offsets and add the negative sum of them once at the start of the next walk phase.

  // Work out incremental forward, left, and turn values for next time step
  Pose2D offset =
    Pose2D(
      (turnRL - prevTurn) * (isLeftSwingFoot ? 1.f : -1.f),
      (isLeftSwingFoot ? -(rightFootOffset.x - prevRightFootOffset.x) : -(leftFootOffset.x - prevLeftFootOffset.x)) * mmPerM,
      (isLeftSwingFoot ? -(rightFootOffset.y - prevRightFootOffset.y) : -(leftFootOffset.y - prevLeftFootOffset.y)) * mmPerM)
      .elementwiseMul(odometryScale);

  // backup values for next computation
  prevLeftFootOffset = leftFootOffset;
  prevRightFootOffset = rightFootOffset;
  prevTurn = turnRL;

  return offset;
}

void Walk2022Generator::compensateArmPosition(const Pose3D& leftFoot, const Pose3D& rightFoot, JointRequest& jointRequest) {

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
    ForwardKinematic::calculateLegChain(Legs::left, temp.jointAngles, theRobotDimensions, balanced.limbs);
    ForwardKinematic::calculateLegChain(Legs::right, temp.jointAngles, theRobotDimensions, balanced.limbs);
    balanced.updateCenterOfMass(theMassCalibration);
    Eigen::Vector3f CoM(balanced.centerOfMass.x, balanced.centerOfMass.y, balanced.centerOfMass.z);
    Eigen::Vector3f balancedCom = torsoRotation * CoM;
    torsoTilt += (balancedCom.x() - withWalkGeneratorArms.centerOfMass.x) * pComFactor;
  }
  float tiltIncrease = forwardTiltOffset;
  VERIFY(
    InverseKinematic::calcLegJoints(
      leftFoot, rightFoot, {0.f, tiltIncrease - torsoTilt * comTiltFactor}, jointRequest.jointAngles, theRobotDimensions) ||
    SystemCall::getMode() == SystemCall::logfileReplay);
}
