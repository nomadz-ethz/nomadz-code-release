/**
 * @file Walk2022Generator.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
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

Walk2022Generator::Walk2022Generator()
    : oscillationController(theOrientationData, theInertiaSensorData, theRobotModel, theRobotDimensions),
      zmpController(thePlannedSteps, theWalkGeneratorData, theRobotDimensions, theRobotModel) {
  InMapFile stream("walk2022GeneratorParams.cfg");
  if (stream.exists()) {
    stream >> static_cast<Walk2022GeneratorParams&>(*this);
  }
  torsoOffset = torsoOffsetBase;
}

void Walk2022Generator::update(WalkGeneratorData& generator) {
  DECLARE_DEBUG_DRAWING3D("module:Walk2022Generator:nextStep", "robot");
  DECLARE_DEBUG_DRAWING3D("module:Walk2022Generator:zmpController", "field");
  DECLARE_PLOT("module:Walk2022Generator:requestedFootLeftX");
  DECLARE_PLOT("module:Walk2022Generator:requestedFootLeftY");
  DECLARE_PLOT("module:Walk2022Generator:requestedFootRightX");

  DECLARE_PLOT("module:Walk2022Generator:ReqCoMX");
  DECLARE_PLOT("module:Walk2022Generator:torsoTilt");
  DECLARE_PLOT("module:Walk2022Generator:requestedFootRightY");
  DECLARE_PLOT("module:Walk2022Generator:originOffsetX");
  DECLARE_PLOT("module:Walk2022Generator:originOffsetY");

  DECLARE_PLOT("module:Walk2022Generator:trajX");
  DECLARE_PLOT("module:Walk2022Generator:trajY");
  DECLARE_PLOT("module:Walk2022Generator:trajZ");

  MODIFY("parameters:Walk2022Generator", static_cast<Walk2022GeneratorParams&>(*this));
  // Use other parameters in demo games to take care of the robots

  measuredFoot[Leg::left] = theRobotModel.limbs[MassCalibration::footLeft];
  measuredFoot[Leg::right] = theRobotModel.limbs[MassCalibration::footRight];
  generator.reset = [this, &generator]() { reset(generator); };

  generator.calcJoints = [this, &generator](Pose2D& speed, Pose2D& target, WalkGeneratorData::WalkMode walkMode) {
    calcJoints(generator, speed, target, walkMode);
  };
  oscillationController.predictOscillationUntilSwitch(generator);

  filteredGyroX = gyroLowPassRatio * filteredGyroX + (1.f - gyroLowPassRatio) * theInertiaSensorData.gyro.x;
  filteredGyroY = gyroLowPassRatio * filteredGyroY + (1.f - gyroLowPassRatio) * theInertiaSensorData.gyro.y;
}

void Walk2022Generator::reset(WalkGeneratorData& generator) {
  generator.stepDuration = 0.f;
  generator.t = 0.f;
  generator.upcomingOdometryOffset = Pose2D();
  generator.walkState = WalkGeneratorData::standing;
  timeWhenStandBegan = theFrameInfo.time;
  if (theRobotInfo.penalty != PENALTY_NONE && theRobotInfo.penalty != PENALTY_SPL_ILLEGAL_MOTION_IN_SET) {
    filteredGyroX = filteredGyroY = 0;
  }

  currentPlannedSpeed = Pose2D();
  for (int i = 0; i < Leg::numOfSides; ++i) {
    float swingFootSign = (i == 0) ? 1.f : -1.f;
    generator.stepTraj[i].foot[StepTraj::refPoint] =
      Vector3<>(-torsoOffset, swingFootSign * (theRobotDimensions.yHipOffset + additionalYOffset), -walkHipHeight);
    prevFootFinal[i].translation = generator.stepTraj[i].foot[StepTraj::refPoint];
    for (auto traj :
         {StepTraj::currentOffset, StepTraj::initialOffset, StepTraj::correctedOffset, StepTraj::combinedOffset}) {

      generator.stepTraj[i].foot[traj] = Vector3<>();
      generator.stepTraj[i].turnRL[traj] = 0.f;
    }
  }
  generator.switchPhase = 0.f;

  generator.swingControlPoint = Vector3<>();
  torsoTilt = 0_deg;
}

void Walk2022Generator::calcNextStep(WalkGeneratorData& generator) {

  for (int i = 0; i < Leg::numOfSides; ++i) {
    generator.measuredCurrentStep[i] = measuredFootPos[i];
  }
  thePlannedSteps.calcStepPattern(generator, requestedSpeed, requestedTarget, requestedWalkMode);
  currentPlannedSpeed = thePlannedSteps.currentPlannedSpeed;

  generator.stepDuration = thePlannedSteps.stepDuration;
  singleSupportPhase = Rangef(0.f, generator.stepDuration);
  doubleSupportPhase = Rangef(generator.stepDuration, generator.stepDuration + zmpController.transitionPhaseDuration);

  generator.stepCorrection = Pose2D();

  newTargetGait[Leg::left] = thePlannedSteps.nextStep[Leg::left];
  newTargetGait[Leg::right] = thePlannedSteps.nextStep[Leg::right];
  // float stepHeightModifier = (thePlannedSteps.isCurrentStepPrepare) ? 1.5f : 1.f;
  Leg::Side swingSide = generator.isLeftPhase ? Leg::left : Leg::right;
  Vector2<> swingDirection =
    newTargetGait[swingSide].translation - generator.stepTraj[swingSide].foot[StepTraj::initialOffset].toVec2();
  swingDirection.normalize();
  swingDirection *= 60.0 * theMotionRequest.walkRequest.inWalkKickRequest.kickStrength;
  generator.swingControlPoint =
    (thePlannedSteps.isCurrentStepKick)
      ? Vector3<>(swingDirection.x, swingDirection.y, calcStepHeight())
      : Vector3<>(0.0, 0.0, calcStepHeight()); // + Vector3<>(currentPlannedSpeed.translation) / 10;
  useBezierCurve = thePlannedSteps.isCurrentStepKick;

  // float speedDiff = currentPlannedSpeed.translation.x - prevPlannedSpeed.translation.x;
  // torsoOffsetTarget = torsoOffsetBase - ((std::abs(speedDiff) > 50.f) ? speedDiff / 10.f : 0.f);
  torsoOffset = torsoOffsetBase;
  correctionApplied = false;

  globPosAtGaitSwitch = theRobotPose;
  globPosAtGaitSwitch.translation = globPosAtGaitSwitch * Vector2<>(torsoOffset, 0.f);
  if (enableZMP) {
    zmpController.update(supportSide);
    for (auto& com : zmpController.comRef) {
      com.y *= augmentComRef;
    }
    calcZMPOffset(generator);
  }
  prevPlannedSpeed = currentPlannedSpeed;
}

void Walk2022Generator::calcJoints(WalkGeneratorData& generator,
                                   Pose2D& reqSpeed,
                                   Pose2D& reqTarget,
                                   WalkGeneratorData::WalkMode reqWalkMode) {
  updateSettings(generator, reqSpeed, reqTarget, reqWalkMode);
  if (generator.t == 0) {
    calcNextStep(generator);
  }
  // 2. Update timer
  generator.t += Constants::motionCycleTime;
  recomputeOptions(generator);
  calcTraj(generator);
  calcOdometryOffset(generator);
  updateLegJoints(generator);
  customDebugDrawing(generator);

  setStiffness(generator);
  updateArmJoints(generator);

  applyGyroBalance(generator);
  checkSwitchCondition(generator);
}

void Walk2022Generator::updateSettings(WalkGeneratorData& generator,
                                       Pose2D& reqSpeed,
                                       Pose2D& reqTarget,
                                       WalkGeneratorData::WalkMode reqWalkMode) {
  // 1. Read in new walk values (forward, left, turn, power) only at the start of a walk step cycle, ie when t = 0
  swingSide = (generator.isLeftPhase) ? Leg::left : Leg::right;
  supportSide = (generator.isLeftPhase) ? Leg::right : Leg::left;
  generator.maxSpeed = thePlannedSteps.maxSpeed;
  requestedSpeed = reqSpeed;
  requestedTarget = reqTarget;
  requestedWalkMode = reqWalkMode;
  // generator.originOffset = originOffset;
  for (int i = 0; i < Leg::numOfSides; ++i) {
    measuredFootPos[i] = Pose2D(measuredFoot[i].rotation.getZAngle(), measuredFoot[i].translation.toVec2());
    float swingFootSign = (i == 0) ? 1.f : -1.f;
    generator.stepTraj[i].foot[StepTraj::refPoint] =
      Vector3<>(-torsoOffset, swingFootSign * (theRobotDimensions.yHipOffset + additionalYOffset), -walkHipHeight);
  }
}

void Walk2022Generator::recomputeOptions(WalkGeneratorData& generator) {
  // 3. Determine Walk2022 Option
  if (generator.walkState != WalkGeneratorData::standing && currentPlannedSpeed == Pose2D()) {
    generator.walkState = WalkGeneratorData::stopping;
  } else if (generator.walkState != WalkGeneratorData::walking && currentPlannedSpeed != Pose2D()) {
    generator.walkState = WalkGeneratorData::starting;
    if (currentPlannedSpeed.translation.y != 0.f && generator.t == 0) {
      generator.isLeftPhase = !(currentPlannedSpeed.translation.y > 0);
      swingSide = (generator.isLeftPhase) ? Leg::left : Leg::right;
      supportSide = (generator.isLeftPhase) ? Leg::right : Leg::left;
      calcNextStep(generator);
    }
  }
}

void Walk2022Generator::calcTraj(WalkGeneratorData& generator) {
  if (generator.walkState == WalkGeneratorData::standing) {
    generator.stepDuration = generator.t = 0.f;
    generator.stepTraj[Leg::left].foot[finalTrajectoryType].z = generator.stepTraj[Leg::right].foot[finalTrajectoryType].z =
      0; // don't move on starting
  } else {
    // 5.3 Calculate intra-walkphase forward, left and turn at time-step dt
    generateStepTrajectory(generator,
                           thePlannedSteps.nextStep[Leg::left],
                           thePlannedSteps.nextStep[Leg::right],
                           StepTraj::currentOffset,
                           useBezierCurve);

    if (generator.stepCorrection != Pose2D() ||
        (generator.t <= generator.stepDuration * stepCorrectionAllowPhaseRange.min)) {
      if (!correctionApplied) {
        targetOriginOffset = Vector3<>(0.f, (generator.isLeftPhase) ? 20.f : -20.f, 0.f);
        correctionPhase = generator.t;
      }
      correctionApplied = true;

      newTargetGait[swingSide] = thePlannedSteps.nextStep[swingSide].elementwiseAdd(generator.stepCorrection.scale(1.f / 2));
      newTargetGait[supportSide] =
        thePlannedSteps.nextStep[supportSide].elementwiseAdd(generator.stepCorrection.scale(-1.f / 2));
    }
    generateStepTrajectory(
      generator, newTargetGait[Leg::left], newTargetGait[Leg::right], StepTraj::correctedOffset, useBezierCurve);
    linearInterpolate(generator, correctionPhase, stepCorrectionAllowPhaseRange.max);
    // 5.4 Special
    // conditions when priming the walk forward check is not needed, because forward is clipped with the maxAcceleration
    // check is used to decide, if the robot can execute a real step as a starting step
    if (generator.walkState == WalkGeneratorData::starting) {
      for (int i = 0; i < Leg::numOfSides; ++i) {
        generator.stepTraj[i].foot[finalTrajectoryType].z *=
          footLiftFirstStepFactor; // reduce max lift due to short duration
        generator.stepTraj[i].foot[finalTrajectoryType].x = generator.stepTraj[i].foot[finalTrajectoryType].y =
          0; // don't move on starting
        generator.stepTraj[i].turnRL[finalTrajectoryType] = 0;
      }
    }
    if (enableZMP) {
      generateOriginOffset(generator);
    } else {
      generator.originOffset = Vector3<>(0.f, 0.f, 0.f);
    }
  }
}

void Walk2022Generator::generateOriginOffset(WalkGeneratorData& generator) {

  generator.originOffset.y = offsetFromDefault[2].y + theRobotModel.centerOfMass.y;
  generator.originOffset.x = offsetFromDefault[0].x + theRobotModel.centerOfMass.x;

  generator.lastOriginOffset.x = offsetFromDefault[0].x;
  generator.lastOriginOffset.y = offsetFromDefault[1].y;
  currentComRef = Vector2<>(zmpController.comRef.front());
  originRef = Vector2<>(zmpController.originRef.front().translation);
  currentZMP = Vector2<>(zmpController.zmp.front());

  offsetFromDefault.erase(offsetFromDefault.begin());
  zmpController.originRef.erase(zmpController.originRef.begin());
  zmpController.comRef.erase(zmpController.comRef.begin());
  zmpController.zmp.erase(zmpController.zmp.begin());
}

void Walk2022Generator::calcZMPOffset(WalkGeneratorData& generator) {
  int calcHorizon = std::ceil((generator.stepDuration + zmpController.transitionPhaseDuration) / Constants::motionCycleTime);
  offsetFromDefault.clear();
  for (int i = 0; i < calcHorizon + 3; ++i) {
    Vector2<> offset = zmpController.originRef[i].invert() * zmpController.comRef[i];
    offsetFromDefault.push_back(offset);
  }
}

void Walk2022Generator::calcGyroOffset(WalkGeneratorData& generator) {
  Vector2<> angularPos = Vector2<>(theOrientationData.rotation.getYAngle(), theOrientationData.rotation.getXAngle());
  Vector2<> angularVel = Vector2<>(theInertiaSensorData.gyro.y, -theInertiaSensorData.gyro.x);
  angularPos.x = angularVel.x = 0.f;
  gyroCorrectionStates[2] = angularPos * gyroCompensationFactorP + angularVel * gyroCompensationFactorD;
  gyroCorrectionStates[1] += gyroCorrectionStates[2] * Constants::motionCycleTime;
  gyroCorrectionStates[0] += (gyroCorrectionStates[1] * Constants::motionCycleTime +
                              gyroCorrectionStates[2] * std::pow(Constants::motionCycleTime, 2));
}

void Walk2022Generator::updateLegJoints(WalkGeneratorData& generator) {
  PLOT("module:Walk2022Generator:trajX", generator.stepTraj[Leg::left].foot[finalTrajectoryType].x);
  PLOT("module:Walk2022Generator:trajY", generator.stepTraj[Leg::left].foot[finalTrajectoryType].y);
  PLOT("module:Walk2022Generator:trajZ", generator.stepTraj[Leg::left].foot[finalTrajectoryType].z);
  for (int i = 0; i < Leg::numOfSides; ++i) {
    prevFootFinal[i] = generator.stepTraj[i].footFinal;
    generator.stepTraj[i].footFinal = Pose3D(generator.stepTraj[i].foot[StepTraj::refPoint] - generator.originOffset)
                                        .translate(generator.stepTraj[i].foot[finalTrajectoryType])
                                        .rotateZ(generator.stepTraj[i].turnRL[finalTrajectoryType]);
  }
  Vector3<> delta = generator.stepTraj[supportSide].footFinal.translation - prevFootFinal[supportSide].translation;
  delta *= deltaScaler;
  augmentedFootFinal[supportSide] = generator.stepTraj[supportSide].footFinal;
  augmentedFootFinal[supportSide].translation =
    generator.stepTraj[supportSide].footFinal.translation + Vector3<>(0.f, delta.y, 0.f);
  augmentedFootFinal[swingSide] = generator.stepTraj[swingSide].footFinal;
  if (enableGyroCompensation) {
    // 9.2 Gyro compensation (only for the swing foot
    calcGyroOffset(generator);
    generator.stepTraj[supportSide].footFinal.translation -= Vector3<>(gyroCorrectionStates[0]);
  }
  // 9.3 Inverse kinematics
  PLOT("module:Walk2022Generator:originOffsetX", generator.originOffset.x);
  PLOT("module:Walk2022Generator:originOffsetY", generator.originOffset.y);
  PLOT("module:Walk2022Generator:requestedFootLeftX", generator.stepTraj[Leg::left].footFinal.translation.x);
  PLOT("module:Walk2022Generator:requestedFootLeftY", generator.stepTraj[Leg::left].footFinal.translation.y);
  PLOT("module:Walk2022Generator:requestedFootRightX", generator.stepTraj[Leg::right].footFinal.translation.x);
  PLOT("module:Walk2022Generator:requestedFootRightY", generator.stepTraj[Leg::right].footFinal.translation.y);
  VERIFY(InverseKinematic::calcLegJoints(augmentedFootFinal[Leg::left],
                                         augmentedFootFinal[Leg::right],
                                         {0, 0},
                                         generator.jointRequest.jointAngles,
                                         theRobotDimensions) ||
         SystemCall::getMode() == SystemCall::logfileReplay);
}

void Walk2022Generator::setStiffness(WalkGeneratorData& generator) {
  // 10. Set joint values and stiffness
  int stiffness =
    generator.walkState == WalkGeneratorData::standing && theFrameInfo.getTimeSince(timeWhenStandBegan) > standStiffnessDelay
      ? HardnessData::useDefault
      : walkStiffness;
  for (uint8_t i = JointData::FirstLegJoint; i < JointData::numOfJoints; ++i) {
    generator.jointRequest.jointHardness.hardness[i] = stiffness;
  }
  // Head can move freely
  generator.jointRequest.jointAngles.angles[JointData::HeadPitch] =
    generator.jointRequest.jointAngles.angles[JointData::HeadYaw] = JointData::ignore;
}

void Walk2022Generator::checkSwitchCondition(WalkGeneratorData& generator) {
#ifdef TARGET_SIM
  if (generator.t > doubleSupportPhase.max) {
    generator.isLeftPhase = !generator.isLeftPhase;
    stateCheck(generator);
  }
#endif
  // 6. Changing Support Foot. Note isLeftPhase means left foot is swing foot.
  // t>0.75*T tries to avoid bounce, especially when side-stepping
  // lastZMPL*ZMPL<0.0 indicates that support foot has changed
  // t>3*T tires to get out of "stuck" situations
  // a predicted foot support switch is used, if the current rotation speed is low
  Rangef supportSwitchRange = Rangef(supportSwitchPhaseRange.min, supportSwitchPhaseRange.max) * generator.stepDuration;
  if ((generator.t > supportSwitchRange.min && theFootSupport.switched) || (generator.t > supportSwitchRange.max)) {
    // if the support switch is predicted, weightShiftStatus and isLeftPhase must be set differently
    const bool usePrediction =
      theFootSupport.predictedSwitched && !theFootSupport.switched && (generator.t <= supportSwitchRange.max);
    generator.usedPredictedSwitch = usePrediction;
    generator.switchPhase = generator.t;

    generator.isLeftPhase = usePrediction ? !generator.isLeftPhase : theFootSupport.support < 0;

    stateCheck(generator);
  }
}

void Walk2022Generator::stateCheck(WalkGeneratorData& generator) {
  if (generator.walkState != WalkGeneratorData::standing) {
    for (int i = 0; i < Leg::numOfSides; ++i) {
      generator.measuredInitialStep[i] = generator.measuredCurrentStep[i];
    }
    for (int i = 0; i < Leg::numOfSides; ++i) {
      generator.stepTraj[i].foot[StepTraj::initialOffset] = generator.stepTraj[i].foot[finalTrajectoryType];
      generator.stepTraj[i].turnRL[StepTraj::initialOffset] = generator.stepTraj[i].turnRL[finalTrajectoryType];
    }
    generator.t = 0;
  }
  switch (generator.walkState) {
  case WalkGeneratorData::standing:

    break;
  case WalkGeneratorData::starting:
    generator.walkState = WalkGeneratorData::walking;
    break;

  case WalkGeneratorData::walking:
    break;

  case WalkGeneratorData::stopping:
    if ((std::abs(generator.stepTraj[Leg::left].turnRL[finalTrajectoryType]) < thresholdStopStandTransition.rotation &&
         std::abs(generator.stepTraj[Leg::left].foot[finalTrajectoryType].x) < thresholdStopStandTransition.translation.x) ||
        theGroundContactState.contact) {
      generator.walkState = WalkGeneratorData::standing;
      timeWhenStandBegan = theFrameInfo.time;
    }
    break;

  default:
    break;
  }
}

void Walk2022Generator::applyGyroBalance(WalkGeneratorData& generator) {
  // 7. Sagittal balance
  Angle balanceAdjustment =
    generator.walkState == WalkGeneratorData::standing
      ? 0.f
      : filteredGyroY * (filteredGyroY > 0 ? gyroForwardBalanceFactor
                                           : gyroBackwardBalanceFactor); // adjust ankle tilt in proportion to filtered gryoY
  generator.jointRequest.jointAngles.angles[generator.isLeftPhase ? JointData::RAnklePitch : JointData::LAnklePitch] +=
    balanceAdjustment;

  // Lateral balance
  if (generator.walkState == WalkGeneratorData::standing) {
    balanceAdjustment = filteredGyroX * gyroSidewaysBalanceFactor;
    generator.jointRequest.jointAngles.angles[JointData::LAnkleRoll] += balanceAdjustment;
    generator.jointRequest.jointAngles.angles[JointData::RAnkleRoll] += balanceAdjustment;
  }
}

float Walk2022Generator::calcStepHeight() {
  return baseFootLift + std::abs(currentPlannedSpeed.rotation) * footLiftIncreaseFactor.rotation +
         std::abs(currentPlannedSpeed.translation.x) * footLiftIncreaseFactor.translation.x +
         std::abs(currentPlannedSpeed.translation.y) * footLiftIncreaseFactor.translation.y;
}

void Walk2022Generator::calcOdometryOffset(WalkGeneratorData& generator) {
  generator.odometryOffset = measuredPrevFootPos[supportSide].elementwiseSub(measuredFootPos[supportSide]);
  for (int i = 0; i < Leg::numOfSides; ++i) {
    measuredPrevFootPos[i] = measuredFootPos[i];
  }
  generator.currentSpeed = generator.odometryOffset.scale(1.f / Constants::motionCycleTime);
}

void Walk2022Generator::updateArmJoints(WalkGeneratorData& generator) {
  // 10.1 Arms
  for (int i = 0; i < 6; ++i) {
    generator.jointRequest.jointAngles.angles[JointData::FirstLeftArmJoint + i] =
      theArmMotionEngineOutput.arms[ArmMotionRequest::left].angles[i];
    generator.jointRequest.jointHardness.hardness[JointData::FirstLeftArmJoint + i] =
      theArmMotionEngineOutput.arms[ArmMotionRequest::left].hardness[i];
  }
  for (int i = 0; i < 6; ++i) {
    generator.jointRequest.jointAngles.angles[JointData::FirstRightArmJoint + i] =
      theArmMotionEngineOutput.arms[ArmMotionRequest::right].angles[i];
    generator.jointRequest.jointHardness.hardness[JointData::FirstRightArmJoint + i] =
      theArmMotionEngineOutput.arms[ArmMotionRequest::right].hardness[i];
  }
  compensateArmPosition(generator);
}

void Walk2022Generator::compensateArmPosition(WalkGeneratorData& generator) {
  // Compensate arm position's effect on COM by tilting torso
  Pose3D& leftFoot = augmentedFootFinal[Leg::left];
  Pose3D& rightFoot = augmentedFootFinal[Leg::right];
  JointRequest& jointRequest = generator.jointRequest;

  JointRequest temp = jointRequest;
  Angle tempTorsoTilt = 0.f;
  for (int joint = 0; joint < JointData::FirstLegJoint; ++joint) {
    temp.jointAngles.angles[joint] = (jointRequest.jointAngles.angles[joint] != JointData::off)
                                       ? theJointRequest.jointAngles.angles[joint]
                                       : theFilteredJointData.angles[joint];
  }
  RobotModel balanced(temp.jointAngles, theRobotDimensions, theMassCalibration);
  Eigen::Vector3f balancedCom;
  for (int i = 0; i < numOfComIterations; ++i) {
    Quaternionf torsoRotation = Rotation::aroundY(tempTorsoTilt);
    VERIFY(InverseKinematic::calcLegJoints(leftFoot, rightFoot, torsoRotation, temp.jointAngles, theRobotDimensions) ||
           SystemCall::getMode() == SystemCall::logfileReplay);
    ForwardKinematic::calculateLegChain(Leg::left, temp.jointAngles, theRobotDimensions, balanced.limbs);
    ForwardKinematic::calculateLegChain(Leg::right, temp.jointAngles, theRobotDimensions, balanced.limbs);
    balanced.updateCenterOfMass(theMassCalibration);
    Eigen::Vector3f CoM(balanced.centerOfMass.x, balanced.centerOfMass.y, balanced.centerOfMass.z);
    balancedCom = torsoRotation * CoM;
    tempTorsoTilt += (6.f - balancedCom.x()) * comTiltFactor;
  }

  torsoTilt =
    (std::abs(torsoTilt - tempTorsoTilt) > fromDegrees(1.f)) ? Angle(0.1 * tempTorsoTilt + 0.9 * torsoTilt) : torsoTilt;
  PLOT("module:Walk2022Generator:ReqCoMX", balancedCom.x());
  PLOT("module:Walk2022Generator:torsoTilt", toDegrees(torsoTilt));
  VERIFY(InverseKinematic::calcLegJoints(leftFoot,
                                         rightFoot,
                                         {0.f, (std::abs(torsoTilt) > fromDegrees(1.f)) ? torsoTilt : Angle(0.f)},
                                         jointRequest.jointAngles,
                                         theRobotDimensions) ||
         SystemCall::getMode() == SystemCall::logfileReplay);
}

void Walk2022Generator::customDebugDrawing(WalkGeneratorData& generator) {
  float plotHeight = 0.f;
  COMPLEX_DRAWING3D("module:Walk2022Generator:zmpController", {
    // Origin Reference Base Point
    for (size_t i = 0; i < zmpController.originBasePoints.size(); ++i) {
      Vector3<> point = Vector3<>(globPosAtGaitSwitch * zmpController.originBasePoints[i].point.translation);
      point.z = plotHeight;
      SPHERE3D_VEC("module:Walk2022Generator:zmpController", point, 2, ColorClasses::red1);
    }
    // Origin Reference
    Vector3<> point = Vector3<>(globPosAtGaitSwitch * originRef);
    SPHERE3D_VEC("module:Walk2022Generator:zmpController", point, 2, ColorClasses::red1);
    for (size_t i = 0; i < zmpController.originRef.size(); ++i) {
      Vector3<> point = Vector3<>(globPosAtGaitSwitch * zmpController.originRef[i].translation);
      point.z = plotHeight - i * shiftPlotPointVertical;
      SPHERE3D_VEC("module:Walk2022Generator:zmpController", point, 1, ColorClasses::red3);
    }

    // ZMP Reference Base Point
    for (size_t i = 0; i < zmpController.zmpRefBasePoints.size(); ++i) {
      Vector3<> point = Vector3<>(globPosAtGaitSwitch * zmpController.zmpRefBasePoints[i].point.translation);
      point.z = plotHeight;
      SPHERE3D_VEC("module:Walk2022Generator:zmpController", point, 1, ColorClasses::purple);
      if (i < zmpController.zmpRefBasePoints.size() - 1) {
        Vector3<> nextPoint = Vector3<>(globPosAtGaitSwitch * zmpController.zmpRefBasePoints[i + 1].point.translation);
        nextPoint.z = plotHeight;
        LINE3D_VEC("module:Walk2022Generator:zmpController", point, nextPoint, 3, ColorClasses::purple);
      }
    }
    // ZMP Reference
    // for (size_t i = 0; i < zmpController.zmpRef.size(); ++i) {
    //   Vector3<> point = Vector3<>(globPosAtGaitSwitch * zmpController.zmpRef[i]);
    //   point.z = plotHeight;
    //   SPHERE3D_VEC("module:Walk2022Generator:zmpController", point, 1, ColorClasses::purple3);
    // }

    // ZMP
    point = Vector3<>(globPosAtGaitSwitch * currentZMP);
    SPHERE3D_VEC("module:Walk2022Generator:zmpController", point, 2, ColorClasses::purple3);
    for (size_t i = 0; i < zmpController.zmp.size(); ++i) {
      Vector3<> point = Vector3<>(globPosAtGaitSwitch * zmpController.zmp[i]);
      point.z = plotHeight - i * shiftPlotPointVertical;
      SPHERE3D_VEC("module:Walk2022Generator:zmpController", point, 1, ColorClasses::purple3);
    }

    // CoM Reference
    point = Vector3<>(globPosAtGaitSwitch * currentComRef);
    SPHERE3D_VEC("module:Walk2022Generator:zmpController", point, 2, ColorClasses::purple3);
    for (size_t i = 0; i < zmpController.comRef.size(); ++i) {
      Vector3<> point = Vector3<>(globPosAtGaitSwitch * zmpController.comRef[i]);
      point.z = plotHeight - i * shiftPlotPointVertical;
      SPHERE3D_VEC("module:Walk2022Generator:zmpController", point, 1, ColorClasses::purple1);
    }
  });

  COMPLEX_DRAWING3D("module:Walk2022Generator:nextStep", {
    for (int i = 0; i < Leg::numOfSides; ++i) {
      debugTraj[i].push_back(
        std::make_pair(generator.stepTraj[i].turnRL[finalTrajectoryType], generator.stepTraj[i].footFinal.translation));
      if (debugTraj[i].size() > 40) {
        debugTraj[i].erase(debugTraj[i].begin());
      }
      const float directionPointOffset = 35.f;
      SPHERE3D_VEC("module:Walk2022Generator:nextStep",
                   Vector3<>(measuredFoot[i].translation + Vector3<>(0.0f, 0.0f, -theRobotDimensions.heightLeg5Joint)),
                   2,
                   ColorClasses::red);
      LINE3D_VEC(
        "module:Walk2022Generator:nextStep",
        Vector3<>(measuredFoot[i].translation +
                  measuredFoot[i].rotation * Vector3<>(0.0f, 0.0f, -theRobotDimensions.heightLeg5Joint)),
        Vector3<>(measuredFoot[i].translation +
                  measuredFoot[i].rotation * Vector3<>(directionPointOffset, 0.0f, -theRobotDimensions.heightLeg5Joint)),
        3,
        ColorClasses::red);

      // Requested Trajectory
      int colorAlphaIndex = 0;
      for (auto& pair : debugTraj[i]) {
        SPHERE3D_VEC("module:Walk2022Generator:nextStep",
                     pair.second,
                     1,
                     ColorRGBA(0, 255, 0, (int)(255 * colorAlphaIndex / debugTraj[i].size())));
        Vector3<> direction =
          pair.second + Vector3<>(std::cos(pair.first), std::sin(pair.first), 0.0) * directionPointOffset;
        SPHERE3D_VEC("module:Walk2022Generator:nextStep",
                     direction,
                     1,
                     ColorRGBA(0, 255, 0, (int)(255 * colorAlphaIndex / debugTraj[i].size())));
        colorAlphaIndex++;
      }
      SPHERE3D_VEC("module:Walk2022Generator:nextStep", generator.stepTraj[i].footFinal.translation, 2, ColorClasses::green);
      Vector3<> directionPoint = generator.stepTraj[i].footFinal.translation +
                                 generator.stepTraj[i].footFinal.rotation * Vector3<>(directionPointOffset, 0.f, 0.f);
      LINE3D_VEC("module:Walk2022Generator:nextStep",
                 generator.stepTraj[i].footFinal.translation,
                 directionPoint,
                 2,
                 ColorClasses::green3);
    }
    SPHERE3D("module:Walk2022Generator:nextStep", 0, 0, -walkHipHeight, 2, ColorClasses::blue);
    SPHERE3D("module:Walk2022Generator:nextStep", -torsoOffset, 0, -walkHipHeight, 1, ColorClasses::blue);
    // Reference Drawings
    SPHERE3D("module:Walk2022Generator:nextStep",
             -torsoOffset,
             theRobotDimensions.yHipOffset,
             -walkHipHeight,
             1,
             ColorClasses::blue);
    SPHERE3D("module:Walk2022Generator:nextStep",
             -torsoOffset,
             -theRobotDimensions.yHipOffset,
             -walkHipHeight,
             1,
             ColorClasses::blue);
    LINE3D("module:Walk2022Generator:nextStep",
           -torsoOffset,
           theRobotDimensions.yHipOffset,
           -walkHipHeight,
           -torsoOffset,
           -theRobotDimensions.yHipOffset,
           -walkHipHeight,
           2,
           ColorClasses::blue);
    LINE3D(
      "module:Walk2022Generator:nextStep", 0, 0, -walkHipHeight, -torsoOffset, -0, -walkHipHeight, 2, ColorClasses::blue);
  });
}
