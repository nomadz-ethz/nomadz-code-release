/**
 * @file MotionCombinator.cpp
 *
 * This file implements a module that combines the motions created by the different modules.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A> and Jesse Richter-Klug
 */

#include "MotionCombinator.h"
#include "Core/SensorData.h"
#include "Core/System/File.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Math/RotationMatrix.h"

#include <iostream>
#include <fstream>

using std::abs;

MAKE_MODULE(MotionCombinator, Motion Control)

MotionCombinator::MotionCombinator() : theNonArmeMotionEngineOutput() {
  currentRecoveryTime = recoveryTime + 1;
  headJawInSavePosition = false;
  headPitchInSavePosition = false;
  isFallingStarted = false;
  fallingFrame = 0;
  wasInBalance = false;
}

void MotionCombinator::update(JointRequest& jointRequest) {
  DECLARE_PLOT("module:MotionCombinator:odometryOffset.x");
  DECLARE_PLOT("module:MotionCombinator:odometryOffset.y");
  DECLARE_PLOT("module:MotionCombinator:odometryOffset.z");
  DECLARE_PLOT("module:MotionCombinator:odometryOffsetAfterCorrection.x");
  DECLARE_PLOT("module:MotionCombinator:odometryOffsetAfterCorrection.y");
  DECLARE_PLOT("module:MotionCombinator:odometryOffsetAfterCorrection.z");
  DECLARE_PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreview.x");
  DECLARE_PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreview.y");
  DECLARE_PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreview.z");
  DECLARE_PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreviewCorrected.x");
  DECLARE_PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreviewCorrected.y");
  DECLARE_PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreviewCorrected.z");

  MODIFY("parameters:MotionCombinator:correctOdometry", correctOdometry);
  MODIFY("parameters:MotionCombinator:saveJoint2File", saveJoint2File);

  specialActionOdometry += theSpecialActionsOutput.odometryOffset;

  const JointRequest* jointRequests[MotionRequest::numOfMotions];
  jointRequests[MotionRequest::walk] = &theWalkingEngineOutput.jointRequest;
  jointRequests[MotionRequest::kick] = &theKickEngineOutput.jointRequest;
  jointRequests[MotionRequest::specialAction] = &theSpecialActionsOutput.jointRequest;

  jointRequest.jointAngles.angles[JointData::HeadYaw] = theHeadJointRequest.pan;
  jointRequest.jointAngles.angles[JointData::HeadPitch] = theHeadJointRequest.tilt;

  copy(*jointRequests[theMotionSelection.targetMotion], jointRequest);

  ASSERT(jointRequest.isValid());

  // Find fully active motion and set MotionInfo
  if (theMotionSelection.ratios[theMotionSelection.targetMotion] == 1.f) {
    Pose2D odometryOffset;
    // default values
    motionInfo.motionRequest.motion = theMotionSelection.targetMotion;
    motionInfo.isMotionStable = true;
    motionInfo.offsetToRobotPoseAfterPreview = Pose2D();
    motionInfo.upcomingOdometryOffset = motionInfo.offsetToRobotPoseAfterPreview;

    lastJointAngles = theJointData;

    switch (theMotionSelection.targetMotion) {
    case MotionRequest::walk: {
      PLOT("module:MotionCombinator:odometryOffset.x", theWalkingEngineOutput.odometryOffset.translation.x);
      PLOT("module:MotionCombinator:odometryOffset.y", theWalkingEngineOutput.odometryOffset.translation.y);
      PLOT("module:MotionCombinator:odometryOffset.r", theWalkingEngineOutput.odometryOffset.rotation);
      if (correctOdometry) {
        odometryOffset = OdometryCorrection::correct(theWalkingEngineOutput.speed,
                                                     theWalkingEngineOutput.odometryOffset,
                                                     theOdometryCorrectionTables.backCorrectionTable,
                                                     theOdometryCorrectionTables.forwardCorrectionTable,
                                                     theOdometryCorrectionTables.sideCorrectionTable,
                                                     theOdometryCorrectionTables.rotCorrectionTable,
                                                     theOdometryCorrectionTables.rot2DCorrectionTable);
        PLOT("module:MotionCombinator:odometryOffsetAfterCorrection.x", odometryOffset.translation.x);
        PLOT("module:MotionCombinator:odometryOffsetAfterCorrection.y", odometryOffset.translation.y);
        PLOT("module:MotionCombinator:odometryOffsetAfterCorrection.r", odometryOffset.rotation);
      } else {
        odometryOffset = theWalkingEngineOutput.odometryOffset;
      }
      motionInfo.motionRequest.walkRequest.mode = theMotionSelection.walkRequest.mode;
      motionInfo.motionRequest.walkRequest.speed = theWalkingEngineOutput.speed;
      // motionInfo.motionRequest.walkRequest.request = theWalkingEngineOutput.speed;
      // motionInfo.motionRequest.walkRequest.requestType = WalkRequest::speedAbs;
      PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreview.x",
           theWalkingEngineOutput.offsetToRobotPoseAfterPreview.translation.x);
      PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreview.y",
           theWalkingEngineOutput.offsetToRobotPoseAfterPreview.translation.y);
      PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreview.r",
           theWalkingEngineOutput.offsetToRobotPoseAfterPreview.rotation);
      motionInfo.offsetToRobotPoseAfterPreview = theWalkingEngineOutput.offsetToRobotPoseAfterPreview;

      if (correctOdometry) {
        motionInfo.upcomingOdometryOffset =
          OdometryCorrection::correctPreview(theWalkingEngineOutput.speed,
                                             theWalkingEngineOutput.offsetToRobotPoseAfterPreview,
                                             theOdometryCorrectionTables.backCorrectionTablePreview,
                                             theOdometryCorrectionTables.forwardCorrectionTablePreview,
                                             theOdometryCorrectionTables.sideCorrectionTablePreview);

        PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreviewCorrected.x",
             motionInfo.offsetToRobotPoseAfterPreview.translation.x);
        PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreviewCorrected.y",
             motionInfo.offsetToRobotPoseAfterPreview.translation.y);
        PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreviewCorrected.r",
             motionInfo.offsetToRobotPoseAfterPreview.rotation);
      } else {
        motionInfo.upcomingOdometryOffset = motionInfo.offsetToRobotPoseAfterPreview;
      }

      break;
    }
    case MotionRequest::kick:
      odometryOffset = theKickEngineOutput.odometryOffset;
      motionInfo.motionRequest.kickRequest = theKickEngineOutput.executedKickRequest;
      motionInfo.isMotionStable = theKickEngineOutput.isStable;
      break;
    case MotionRequest::specialAction:
      odometryOffset = specialActionOdometry;
      specialActionOdometry = Pose2D();
      motionInfo.motionRequest.specialActionRequest = theSpecialActionsOutput.executedSpecialAction;
      motionInfo.isMotionStable = theSpecialActionsOutput.isMotionStable;
      break;
    default:
      break;
    }

    if (theMotionSelection.targetMotion != MotionRequest::walk && theRobotInfo.hasFeature(RobotInfo::zGyro) &&
        (theFallDownState.state == FallDownState::upright)) {
      Vector3<> rotatedGyros = theOrientationData.rotation * theInertiaSensorData.gyro;
      odometryOffset.rotation = rotatedGyros.z * theFrameInfo.cycleTime;
    }

    odometryData += odometryOffset;
    ASSERT(jointRequest.isValid());
  } else // interpolate motions
  {
    const bool interpolateStiffness =
      !(theMotionSelection.targetMotion != MotionRequest::specialAction &&
        theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead &&
        theMotionSelection.ratios[MotionRequest::specialAction] > 0.f); // do not interpolate from play_dead
    for (int i = 0; i < MotionRequest::numOfMotions; ++i) {
      if (i != theMotionSelection.targetMotion && theMotionSelection.ratios[i] > 0) {
        interpolate(*jointRequests[i],
                    *jointRequests[theMotionSelection.targetMotion],
                    theMotionSelection.ratios[i],
                    jointRequest,
                    interpolateStiffness,
                    JointData::HeadYaw,
                    JointData::HeadPitch);
        interpolate(*jointRequests[i],
                    *jointRequests[theMotionSelection.targetMotion],
                    theMotionSelection.ratios[i],
                    jointRequest,
                    interpolateStiffness,
                    JointData::LShoulderPitch,
                    JointData::LHand);
        interpolate(*jointRequests[i],
                    *jointRequests[theMotionSelection.targetMotion],
                    theMotionSelection.ratios[i],
                    jointRequest,
                    interpolateStiffness,
                    JointData::RShoulderPitch,
                    JointData::RHand);
        interpolate(*jointRequests[i],
                    *jointRequests[theMotionSelection.targetMotion],
                    theMotionSelection.ratios[i],
                    jointRequest,
                    interpolateStiffness,
                    JointData::LHipYawPitch,
                    JointData::RAnkleRoll);
      }
    }
  }

  ASSERT(jointRequest.isValid());

  /*auto combinateArmMotions = [&](Arms::Arm const arm)
  {
  const int ratioIndexOffset = arm * theArmMotionSelection.rightArmRatiosOffset;
  const JointData::Joint startJoint = arm == Arms::left ? JointData::LShoulderPitch : JointData::RShoulderPitch;
  const JointData::Joint endJoint = arm == Arms::left ? JointData::LHand : JointData::RHand;

  if(theArmMotionSelection.armRatios[ratioIndexOffset + ArmMotionRequest::none] != 1.f)
  {
  if(theArmMotionSelection.armRatios[ratioIndexOffset + ArmMotionRequest::none] > 0 &&
  ArmMotionRequest::none != theArmMotionSelection.targetArmMotion[arm])
  copy(jointRequest, theNonArmeMotionEngineOutput, startJoint, endJoint);

  if(ArmMotionRequest::none != theArmMotionSelection.targetArmMotion[arm])
  copy(*armJointRequests[theArmMotionSelection.targetArmMotion[arm]], jointRequest, startJoint, endJoint);
  }

  if(theArmMotionSelection.armRatios[ratioIndexOffset + theArmMotionSelection.targetArmMotion[arm]] == 1.f)
  {
  armMotionInfo.armMotion[arm] = theArmMotionSelection.targetArmMotion[arm];

  switch(theArmMotionSelection.targetArmMotion[arm])
  {
  case ArmMotionRequest::keyFrame:
  armMotionInfo.armKeyFrameRequest = theArmMotionSelection.armKeyFrameRequest;
  break;
  case ArmMotionRequest::none:
  default:
  break;
  }
  }
  else
  {
  const bool interpolateStiffness = !(theMotionSelection.targetMotion != MotionRequest::specialAction &&
  theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead &&
  theMotionSelection.ratios[MotionRequest::specialAction] > 0.f &&
  theArmMotionSelection.armRatios[ratioIndexOffset + ArmMotionRequest::none] > 0);

  const JointRequest toJointRequest = theArmMotionSelection.targetArmMotion[arm] == ArmMotionRequest::none ?
  *jointRequests[theMotionSelection.targetMotion] : *armJointRequests[theArmMotionSelection.targetArmMotion[arm]];

  for(int i = 0; i < ArmMotionRequest::numOfArmMotions; ++i)
  {
  if(i != theArmMotionSelection.targetArmMotion[arm] && theArmMotionSelection.armRatios[ratioIndexOffset + i] > 0)
  {
  interpolate(*armJointRequests[i], toJointRequest, theArmMotionSelection.armRatios[ratioIndexOffset + i], jointRequest,
  interpolateStiffness, startJoint, endJoint);
  }
  }
  }

  ASSERT(jointRequest.isValid());
  };

  combinateArmMotions(Arms::left);
  combinateArmMotions(Arms::right);*/

  // TODO: SHOULD BALANCING BE HERE?
  if ((useBalancing) || (usePlayDeadBalancing && theMotionSelection.ratios[MotionRequest::specialAction] == 1.f &&
                         theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead)) {
    balanceUpperBody(jointRequest);
  }

  if (emergencyOffEnabled) {
    if ((theFallDownState.state == FallDownState::onGround)) {
      centerHead(jointRequest);

      ASSERT(jointRequest.isValid());
    } else {
      if (theFallDownState.state == FallDownState::upright) {
        headJawInSavePosition = false;
        headPitchInSavePosition = false;
        isFallingStarted = false;
      }

      if (currentRecoveryTime < recoveryTime) {
        currentRecoveryTime += 1;
        float ratio = (1.f / float(recoveryTime)) * currentRecoveryTime;
        for (int i = 0; i < JointData::numOfJoints; i++) {
          jointRequest.jointHardness.hardness[i] = 30 + int(ratio * float(jointRequest.jointHardness.hardness[i] - 30));
        }
      }
    }
  }

  float sum(0);
  int count(0);
  for (int i = JointData::LHipYawPitch; i < JointData::numOfJoints; i++) {
    if (jointRequest.jointAngles.angles[i] != JointData::off && jointRequest.jointAngles.angles[i] != JointData::ignore &&
        lastJointRequest.jointAngles.angles[i] != JointData::off &&
        lastJointRequest.jointAngles.angles[i] != JointData::ignore) {
      sum += abs(jointRequest.jointAngles.angles[i] - lastJointRequest.jointAngles.angles[i]);
      count++;
    }
  }
  PLOT("module:MotionCombinator:deviations:JointRequest:legsOnly", sum / count);
  for (int i = 0; i < JointData::LHipYawPitch; i++) {
    if (jointRequest.jointAngles.angles[i] != JointData::off && jointRequest.jointAngles.angles[i] != JointData::ignore &&
        lastJointRequest.jointAngles.angles[i] != JointData::off &&
        lastJointRequest.jointAngles.angles[i] != JointData::ignore) {
      sum += abs(jointRequest.jointAngles.angles[i] - lastJointRequest.jointAngles.angles[i]);
      count++;
    }
  }
  // the front getUpMotion requires very high stiffness. Doesn't work with joint compliance.
  if (useDynamicStiffness &&
      (theMotionSelection.ratios[MotionRequest::walk] == 1.f ||
       (theMotionSelection.ratios[MotionRequest::specialAction] == 1.f &&
        (theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::stand ||
         theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::standHigh)))) {
    for (int i = JointData::FirstLegJoint; i < JointData::numOfJoints; i++) {
      float jointErrorNormalized = abs(lastJointRequest.jointAngles.angles[i] - theJointData.angles[i]) / maxJointErrorLimit;
      jointRequest.jointHardness.hardness[i] =
        dynamicStiffnessRange.limit(dynamicStiffnessScaler * jointErrorNormalized + dynamicStiffnessRange.min);
    }
  }
  // Remove Stiffness from hands entirely. This is causing a lot of overheating on some robot with broken fingers.
  jointRequest.jointHardness.hardness[JointData::LHand] = 0;
  jointRequest.jointHardness.hardness[JointData::RHand] = 0;
  PLOT("module:MotionCombinator:deviations:JointRequest:all", sum / count);

  sum = 0;
  count = 0;
  for (int i = JointData::LHipYawPitch; i < JointData::numOfJoints; i++) {
    if (lastJointRequest.jointAngles.angles[i] != JointData::off &&
        lastJointRequest.jointAngles.angles[i] != JointData::ignore) {
      sum += abs(lastJointRequest.jointAngles.angles[i] - theJointData.angles[i]);
      count++;
    }
  }
  PLOT("module:MotionCombinator:differenceToJointData:legsOnly", sum / count);

  for (int i = 0; i < JointData::LHipYawPitch; i++) {
    if (lastJointRequest.jointAngles.angles[i] != JointData::off &&
        lastJointRequest.jointAngles.angles[i] != JointData::ignore) {
      sum += abs(lastJointRequest.jointAngles.angles[i] - theJointData.angles[i]);
      count++;
    }
  }
  lastJointRequest = jointRequest;
  PLOT("module:MotionCombinator:differenceToJointData:all", sum / count);
/*
for(int i = JointData::LHipYawPitch; i < JointData::numOfJoints; i++)
{
char name[100];
sprintf(name, "module:MotionCombinator:angles[%d]", i);
PLOT(name, jointRequest.jointAngles.angles[i]);
sprintf(name, "module:MotionCombinator:angles[%d]", i);
PLOT(name, jointRequest.jointHardness.[i]);
}*/
#ifndef NDEBUG
  if (!jointRequest.isValid()) {
    {
      std::string logDir = "";
#ifdef TARGET_ROBOT
      logDir = "../logs/";
#endif
      OutMapFile stream(logDir + "jointRequest.log");
      stream << jointRequest;
      OutMapFile stream2(logDir + "motionSelection.log");
      stream2 << theMotionSelection;
    }
    ASSERT(false);
  }
#endif
  if (saveJoint2File && (lastSaveJoint2FileValue != saveJoint2File)) {
    saveJoints();
  }
  lastSaveJoint2FileValue = saveJoint2File;
  PLOT("module:MotionCombinator:leftLegAngles[0]", jointRequest.jointAngles.angles[JointData::LHipYawPitch + 0] * _180_pi);
  PLOT("module:MotionCombinator:leftLegAngles[1]", jointRequest.jointAngles.angles[JointData::LHipYawPitch + 1] * _180_pi);
  PLOT("module:MotionCombinator:leftLegAngles[2]", jointRequest.jointAngles.angles[JointData::LHipYawPitch + 2] * _180_pi);
  PLOT("module:MotionCombinator:leftLegAngles[3]", jointRequest.jointAngles.angles[JointData::LHipYawPitch + 3] * _180_pi);
  PLOT("module:MotionCombinator:leftLegAngles[4]", jointRequest.jointAngles.angles[JointData::LHipYawPitch + 4] * _180_pi);
  PLOT("module:MotionCombinator:leftLegAngles[5]", jointRequest.jointAngles.angles[JointData::LHipYawPitch + 5] * _180_pi);

  PLOT("module:MotionCombinator:rightLegAngles[0]", jointRequest.jointAngles.angles[JointData::LHipYawPitch + 6] * _180_pi);
  PLOT("module:MotionCombinator:rightLegAngles[1]", jointRequest.jointAngles.angles[JointData::LHipYawPitch + 7] * _180_pi);
  PLOT("module:MotionCombinator:rightLegAngles[2]", jointRequest.jointAngles.angles[JointData::LHipYawPitch + 8] * _180_pi);
  PLOT("module:MotionCombinator:rightLegAngles[3]", jointRequest.jointAngles.angles[JointData::LHipYawPitch + 9] * _180_pi);
  PLOT("module:MotionCombinator:rightLegAngles[4]", jointRequest.jointAngles.angles[JointData::LHipYawPitch + 10] * _180_pi);
  PLOT("module:MotionCombinator:rightLegAngles[5]", jointRequest.jointAngles.angles[JointData::LHipYawPitch + 11] * _180_pi);

  PLOT("module:MotionCombinator:leftLegStiffness[0]", jointRequest.jointHardness.hardness[JointData::LHipYawPitch + 0]);
  PLOT("module:MotionCombinator:leftLegStiffness[1]", jointRequest.jointHardness.hardness[JointData::LHipYawPitch + 1]);
  PLOT("module:MotionCombinator:leftLegStiffness[2]", jointRequest.jointHardness.hardness[JointData::LHipYawPitch + 2]);
  PLOT("module:MotionCombinator:leftLegStiffness[3]", jointRequest.jointHardness.hardness[JointData::LHipYawPitch + 3]);
  PLOT("module:MotionCombinator:leftLegStiffness[4]", jointRequest.jointHardness.hardness[JointData::LHipYawPitch + 4]);
  PLOT("module:MotionCombinator:leftLegStiffness[5]", jointRequest.jointHardness.hardness[JointData::LHipYawPitch + 5]);

  PLOT("module:MotionCombinator:rightLegStiffness[0]", jointRequest.jointHardness.hardness[JointData::LHipYawPitch + 6]);
  PLOT("module:MotionCombinator:rightLegStiffness[1]", jointRequest.jointHardness.hardness[JointData::LHipYawPitch + 7]);
  PLOT("module:MotionCombinator:rightLegStiffness[2]", jointRequest.jointHardness.hardness[JointData::LHipYawPitch + 8]);
  PLOT("module:MotionCombinator:rightLegStiffness[3]", jointRequest.jointHardness.hardness[JointData::LHipYawPitch + 9]);
  PLOT("module:MotionCombinator:rightLegStiffness[4]", jointRequest.jointHardness.hardness[JointData::LHipYawPitch + 10]);
  PLOT("module:MotionCombinator:rightLegStiffness[5]", jointRequest.jointHardness.hardness[JointData::LHipYawPitch + 11]);

  PLOT("module:MotionCombinator:leftLegAnglesMeasured[0]", theJointData.angles[JointData::LHipYawPitch + 0] * _180_pi);
  PLOT("module:MotionCombinator:leftLegAnglesMeasured[1]", theJointData.angles[JointData::LHipYawPitch + 1] * _180_pi);
  PLOT("module:MotionCombinator:leftLegAnglesMeasured[2]", theJointData.angles[JointData::LHipYawPitch + 2] * _180_pi);
  PLOT("module:MotionCombinator:leftLegAnglesMeasured[3]", theJointData.angles[JointData::LHipYawPitch + 3] * _180_pi);
  PLOT("module:MotionCombinator:leftLegAnglesMeasured[4]", theJointData.angles[JointData::LHipYawPitch + 4] * _180_pi);
  PLOT("module:MotionCombinator:leftLegAnglesMeasured[5]", theJointData.angles[JointData::LHipYawPitch + 5] * _180_pi);

  PLOT("module:MotionCombinator:rightLegAnglesMeasured[0]", theJointData.angles[JointData::RHipYawPitch + 0] * _180_pi);
  PLOT("module:MotionCombinator:rightLegAnglesMeasured[1]", theJointData.angles[JointData::RHipYawPitch + 1] * _180_pi);
  PLOT("module:MotionCombinator:rightLegAnglesMeasured[2]", theJointData.angles[JointData::RHipYawPitch + 2] * _180_pi);
  PLOT("module:MotionCombinator:rightLegAnglesMeasured[3]", theJointData.angles[JointData::RHipYawPitch + 3] * _180_pi);
  PLOT("module:MotionCombinator:rightLegAnglesMeasured[4]", theJointData.angles[JointData::RHipYawPitch + 4] * _180_pi);
  PLOT("module:MotionCombinator:rightLegAnglesMeasured[5]", theJointData.angles[JointData::RHipYawPitch + 5] * _180_pi);
}

void MotionCombinator::update(OdometryData& odometryData) {
  if (!theRobotInfo.hasFeature(RobotInfo::zGyro) || (theFallDownState.state != FallDownState::upright)) {
    this->odometryData.rotation += theFallDownState.odometryRotationOffset;
  }
  this->odometryData.rotation = float(Angle(this->odometryData.rotation).normalize());

  odometryData = this->odometryData;

  Pose2D odometryOffset(odometryData);
  odometryOffset -= lastOdometryData;
  PLOT("module:MotionCombinator:odometryOffsetX", odometryOffset.translation.x);
  PLOT("module:MotionCombinator:odometryOffsetY", odometryOffset.translation.y);
  PLOT("module:MotionCombinator:odometryOffsetRotation", toDegrees(odometryOffset.rotation));
  lastOdometryData = odometryData;
}

void MotionCombinator::saveFall(JointRequest& jointRequest) {
  for (int i = 0; i < JointData::numOfJoints; i++) {
    jointRequest.jointHardness.hardness[i] = 30;
  }
}

void MotionCombinator::centerArms(JointRequest& jointRequest) {
  if (!isFallingStarted) {
    isFallingStarted = true;
    fallingFrame = 0;
  }
}

void MotionCombinator::centerArm(JointRequest& jointRequest, bool left) {
  const int sign(-1 + 2 * left); // sign = 1 for left arm, for right its -1
  int i(left ? JointData::LShoulderPitch : JointData::RShoulderPitch);
  int j(i);

  if (fallingFrame < 20) {
    jointRequest.jointAngles.angles[i++] = 119.5_deg;
    jointRequest.jointAngles.angles[i++] = sign * 25_deg;
    jointRequest.jointAngles.angles[i++] = sign * 45_deg;
    jointRequest.jointAngles.angles[i++] = sign * -11.5_deg;

    while (i < j) {
      jointRequest.jointHardness.hardness[j++] = 100;
    }
  } else if (fallingFrame < 80) {
    jointRequest.jointAngles.angles[i++] = 90_deg;
    jointRequest.jointAngles.angles[i++] = sign * 11.5_deg;
    jointRequest.jointAngles.angles[i++] = sign * -90_deg;
    jointRequest.jointAngles.angles[i++] = sign * -11.5_deg;

    if (fallingFrame < 40) {
      while (i < j) {
        jointRequest.jointHardness.hardness[j++] = 100;
      }
    } else {
      while (j < i) {
        jointRequest.jointHardness.hardness[j++] = 0;
      }
    }
  }
}

void MotionCombinator::centerHead(JointRequest& jointRequest) {
  jointRequest.jointAngles.angles[JointData::HeadYaw] = 0;
  jointRequest.jointAngles.angles[JointData::HeadPitch] = 0;
  if (theFallDownState.direction == FallDownState::front) {
    jointRequest.jointAngles.angles[JointData::HeadPitch] = -0.65f;
  } else if (theFallDownState.direction == FallDownState::back) {
    jointRequest.jointAngles.angles[JointData::HeadPitch] = 0.5f;
  }
  if (abs(theJointData.angles[JointData::HeadYaw]) > 0.1f && !headJawInSavePosition) {
    jointRequest.jointHardness.hardness[JointData::HeadYaw] = 100;
  } else {
    headJawInSavePosition = true;
    jointRequest.jointHardness.hardness[JointData::HeadYaw] = 25;
  }
  if (abs(theJointData.angles[JointData::HeadPitch] - jointRequest.jointAngles.angles[JointData::HeadPitch]) > 0.1f &&
      !headPitchInSavePosition) {
    jointRequest.jointHardness.hardness[JointData::HeadPitch] = 100;
  } else {
    headPitchInSavePosition = true;
    jointRequest.jointHardness.hardness[JointData::HeadPitch] = 50;
  }
}

void MotionCombinator::balanceUpperBody(JointRequest& jointRequest) {
  if (theFallDownState.state != FallDownState::upright) {
    return;
  }
  float newComX = theRobotModel.centerOfMass.x;

  // when going to play dead
  if (usePlayDeadBalancing) {
    if (theMotionSelection.ratios[MotionRequest::specialAction] == 1.f &&
        theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead) {
      if (angleY_playDead != 0 && std::abs(theInertiaSensorData.angle.y) < 20_deg) {
        jointRequest.jointHardness.hardness[JointData::LHipPitch] = hardness_playDead;
        jointRequest.jointHardness.hardness[JointData::RHipPitch] = hardness_playDead;
      }
      jointRequest.jointAngles.angles[JointData::LHipPitch] =
        hipPitch_playDead + theInertiaSensorData.angle.y * angleY_playDead;
      jointRequest.jointAngles.angles[JointData::RHipPitch] =
        hipPitch_playDead + theInertiaSensorData.angle.y * angleY_playDead;
    }
  } else if (!useBalancing) {
    return;
  }

  for (int i = JointData::LHipYawPitch; i <= JointData::RAnkleRoll; i++) {
    if (jointRequest.jointAngles.angles[i] == JointData::ignore || jointRequest.jointAngles.angles[i] == JointData::off) {
      return;
    }
  }

  bool isSpecialAction = theMotionSelection.ratios[MotionRequest::specialAction] == 1.f;
  bool isWalk = theMotionSelection.ratios[MotionRequest::walk] == 1.f;

  if (!isWalk && !isSpecialAction) {
    return;
  }

  BalanceParameters& currentBalanceParams = (isWalk) ? balanceParamsWalk : balanceParams;

  SpecialActionRequest::SpecialActionID specialAction = theMotionSelection.specialActionRequest.specialAction;
  if (isSpecialAction) {
    int startBalanceAfter = -1;
    int timeForSpecialAction = -1;
    Angle maxAngleXForBalance = 0_deg;
    Angle maxAngleYForBalance = 0_deg;
    bool balancedSpecialAction = false;
    for (size_t i = 0; i < specialActionBalanceList.specialActionBalanceEntries.size(); i++) {
      SpecialActionBalanceList::SpecialActionBalanceEntry& entry = specialActionBalanceList.specialActionBalanceEntries[i];
      if (specialAction == entry.specialAction) {
        balancedSpecialAction = true;
        lastBalancedSpecialAction = specialAction;
        startBalanceAfter = entry.balanceStartTime;
        timeForSpecialAction = entry.specialActionDuration;
        maxAngleXForBalance = entry.maxXAngle + (wasInBalance ? 10_deg : 0_deg);
        maxAngleYForBalance = entry.maxYAngle + (wasInBalance ? 10_deg : 0_deg);
        if (theFrameInfo.getTimeSince(timeWhenSpecialActionStarted) > timeForSpecialAction) {
          timeWhenSpecialActionStarted = theFrameInfo.time;
        }
        break;
      }
    }

    if (!balancedSpecialAction) {
      wasInBalance = false;
      timeWhenSpecialActionStarted = 0;
      return;
    }

    int timeSinceSpecialAction = theFrameInfo.getTimeSince(timeWhenSpecialActionStarted);

    // only balance in this time windows, if body angles are within parameter range
    if (timeSinceSpecialAction <= timeForSpecialAction && timeSinceSpecialAction > startBalanceAfter &&
        std::abs(theInertiaSensorData.angle.y) < maxAngleYForBalance &&
        std::abs(theInertiaSensorData.angle.x) < maxAngleXForBalance) {
      // balance!
      wasInBalance = true;
    } else {
      wasInBalance = false;
      return;
    }
  } else {
    wasInBalance = false;
    timeWhenSpecialActionStarted = 0;
  }

  currentBalanceParams = balanceParams;

  Angle targetAngle = currentBalanceParams.targetAngle;
  Angle angleErrorY = (targetAngle - theInertiaSensorData.angle.y);

  float gyroX = theInertiaSensorData.gyro.x;
  float gyroY = theInertiaSensorData.gyro.y;
  float comX = (newComX - lastComX);

  float filteredAngleOffsetX =
    gyroX * currentBalanceParams.gyroX_p * (1 - currentBalanceParams.angleGyroRatioX) +
    theInertiaSensorData.angle.x * currentBalanceParams.angleX_p * currentBalanceParams.angleGyroRatioX;
  float filteredAngleOffsetY =
    gyroY * currentBalanceParams.gyroY_p + angleErrorY * currentBalanceParams.angleY_p + comX * currentBalanceParams.comX_p;
  // sine function like sensor control influence?
  // filteredAngleOffsetY *= std::sin(std::min(pi2, pi2*theInertiaSensorData.angle.y));
  filteredAngleOffsetX += theInertiaSensorData.angle.x * currentBalanceParams.angleX_i;
  filteredAngleOffsetY += angleErrorY * currentBalanceParams.angleY_i;
  filteredAngleOffsetX += gyroX * currentBalanceParams.gyroX_d;
  filteredAngleOffsetY += gyroY * currentBalanceParams.gyroY_d;

  JointRequest old;
  copy(jointRequest, old, JointData::LHipYawPitch);

  jointRequest.jointAngles.angles[JointData::LHipPitch] += filteredAngleOffsetY * currentBalanceParams.hipPitchFactor;
  jointRequest.jointAngles.angles[JointData::RHipPitch] += filteredAngleOffsetY * currentBalanceParams.hipPitchFactor;
  jointRequest.jointAngles.angles[JointData::LKneePitch] += std::abs(filteredAngleOffsetY) * currentBalanceParams.kneeFactor;
  jointRequest.jointAngles.angles[JointData::RKneePitch] += std::abs(filteredAngleOffsetY) * currentBalanceParams.kneeFactor;
  jointRequest.jointAngles.angles[JointData::LAnklePitch] += filteredAngleOffsetY * currentBalanceParams.footPitchFactor;
  jointRequest.jointAngles.angles[JointData::RAnklePitch] += filteredAngleOffsetY * currentBalanceParams.footPitchFactor;

  jointRequest.jointAngles.angles[JointData::LHipRoll] += filteredAngleOffsetX * currentBalanceParams.hipRollFactor;
  jointRequest.jointAngles.angles[JointData::RHipRoll] += filteredAngleOffsetX * currentBalanceParams.hipRollFactor;
  jointRequest.jointAngles.angles[JointData::LAnkleRoll] += filteredAngleOffsetX * currentBalanceParams.footRollFactor;
  jointRequest.jointAngles.angles[JointData::RAnkleRoll] += filteredAngleOffsetX * currentBalanceParams.footRollFactor;

  lastComX = newComX;
}

void MotionCombinator::saveJoints() {
  const std::string joints_file_name = "Joints_sysid.csv";
#ifdef TARGET_ROBOT
  std::ofstream joints_file(std::string("/home/nao/logs/") + joints_file_name, std::ios::app);
#else
  std::ofstream joints_file(std::string(File::getBHDir()) + "/" + joints_file_name, std::ios::app);
#endif
  joints_file << theFrameInfo.time;
  for (int i = 0; i < JointData::numOfJoints; ++i) {
    joints_file << " " << toDegrees(theJointData.angles[i]);
    joints_file << ", " << lastJointRequest.jointHardness.hardness[i];
  }
  joints_file << std::endl;
  joints_file.close();
}

void MotionCombinator::copy(const JointRequest& source,
                            JointRequest& target,
                            const JointData::Joint startJoint,
                            const JointData::Joint endJoint) const {
  for (int i = startJoint; i <= endJoint; ++i) {
    if (source.jointAngles.angles[i] != JointData::ignore) {
      target.jointAngles.angles[i] = source.jointAngles.angles[i];
    }
    target.jointHardness.hardness[i] = target.jointAngles.angles[i] != JointData::off ? source.jointHardness.hardness[i] : 0;
    if (target.jointHardness.hardness[i] == HardnessData::useDefault) {
      target.jointHardness.hardness[i] = theHardnessSettings.hardness[i];
    }
  }
}

void MotionCombinator::interpolate(const JointRequest& from,
                                   const JointRequest& to,
                                   float fromRatio,
                                   JointRequest& target,
                                   bool interpolateStiffness,
                                   const JointData::Joint startJoint,
                                   const JointData::Joint endJoint) const {
  for (int i = startJoint; i <= endJoint; ++i) {
    float f = from.jointAngles.angles[i];
    float t = to.jointAngles.angles[i];

    if (t == JointData::ignore && f == JointData::ignore) {
      continue;
    }

    if (target.jointAngles.angles[i] == JointData::off || target.jointAngles.angles[i] == JointData::ignore) {
      target.jointAngles.angles[i] = lastJointAngles.angles[i];
    }

    if (t == JointData::ignore) {
      t = target.jointAngles.angles[i];
    }
    if (f == JointData::ignore) {
      f = target.jointAngles.angles[i];
    }

    int fStiffness = f != JointData::off ? from.jointHardness.hardness[i] : 0;
    int tStiffness = t != JointData::off ? to.jointHardness.hardness[i] : 0;
    if (fStiffness == HardnessData::useDefault) {
      fStiffness = theHardnessSettings.hardness[i];
    }
    if (tStiffness == HardnessData::useDefault) {
      tStiffness = theHardnessSettings.hardness[i];
    }

    if (t == JointData::off || t == JointData::ignore) {
      t = lastJointAngles.angles[i];
    }
    if (f == JointData::off || f == JointData::ignore) {
      f = lastJointAngles.angles[i];
    }

    if (f == JointData::off) {
      f = t;
    }

    ASSERT(target.jointAngles.angles[i] != JointData::off && target.jointAngles.angles[i] != JointData::ignore);
    ASSERT(f == t || (t != JointData::off && t != JointData::ignore));
    ASSERT(f == t || f != JointData::off && f != JointData::ignore);

    target.jointAngles.angles[i] += -fromRatio * t + fromRatio * f;
    if (interpolateStiffness) {
      target.jointHardness.hardness[i] += int(-fromRatio * float(tStiffness) + fromRatio * float(fStiffness));
    } else {
      target.jointHardness.hardness[i] = tStiffness;
    }
  }
}
