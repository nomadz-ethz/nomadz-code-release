/**
 * @file LimbCombinator.cpp
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#include "LimbCombinator.h"

#define LOGGING
#include "Tools/DortmundWalkingEngine/CSVLogger.h"
#include "Core/Debugging/DebugDrawings.h"

LimbCombinator::LimbCombinator() {
  init = false;
  for (int i = 0; i < JointData::numOfJoints; i++) {
    for (int j = 0; j < 200; j++) {
      angleoffset[i][j] = 0;
    }
  }
}

LimbCombinator::~LimbCombinator() {}

int LimbCombinator::walkingEngineTime = 0;

void LimbCombinator::update(WalkingEngineOutput& walkingEngineOutput) {
  walkingEngineTime++;

  float delayMeasurementOffset = 0;

  MODIFY("delayMeasurementOffset", delayMeasurementOffset);

  if (!init) {
    for (int i = 0; i < JointData::numOfJoints; i++) {
      filter[i].createBuffer(theWalkingEngineParams.outFilterOrder);
    }
    init = true;
  }

  for (int i = 0; i < JointData::numOfJoints; i++) {
    walkingEngineOutput.jointRequest.jointAngles.angles[i] = filter[i].nextValue(theKinematicOutput.jointAngles.angles[i]);
    if (walkingEngineOutput.jointRequest.jointAngles.angles[i] != walkingEngineOutput.jointRequest.jointAngles.angles[i]) {
      ASSERT(walkingEngineOutput.jointRequest.jointAngles.angles[i] ==
             walkingEngineOutput.jointRequest.jointAngles.angles[i]);
    }
  }

  if (theArmMovement.usearms) {
    for (int i = JointData::FirstArmJoint; i < JointData::LHipYawPitch; i++) {
      walkingEngineOutput.jointRequest.jointAngles.angles[i] = theArmMovement.jointAngles.angles[i];
    }
  } else // if theArmMovement.usearms == false
  {
    for (int i = JointData::FirstArmJoint; i < JointData::LHipYawPitch; i++) {
      walkingEngineOutput.jointRequest.jointAngles.angles[i] = JointData::ignore;
    }
  }

  walkingEngineOutput.odometryOffset = theWalkingInfo.odometryOffset;
  walkingEngineOutput.isLeavingPossible = theWalkingInfo.isLeavingPossible;
  walkingEngineOutput.speed.translation.x = theSpeedInfo.speed.translation.x * 1000;
  walkingEngineOutput.speed.translation.y = theSpeedInfo.speed.translation.y * 1000;
  walkingEngineOutput.speed.rotation = theSpeedInfo.speed.rotation;

  walkingEngineOutput.offsetToRobotPoseAfterPreview = theWalkingInfo.offsetToRobotPoseAfterPreview;
  // set leg hardness
  static unsigned int waitCounter = 0;
  {
    walkingEngineOutput.jointRequest.jointHardness.hardness[JointData::LHipYawPitch] =
      theWalkingEngineParams.jointCalibration.legJointHardness[0];
    walkingEngineOutput.jointRequest.jointHardness.hardness[JointData::LHipRoll] =
      theWalkingEngineParams.jointCalibration.legJointHardness[1];
    walkingEngineOutput.jointRequest.jointHardness.hardness[JointData::LHipPitch] =
      theWalkingEngineParams.jointCalibration.legJointHardness[2];
    walkingEngineOutput.jointRequest.jointHardness.hardness[JointData::LKneePitch] =
      theWalkingEngineParams.jointCalibration.legJointHardness[3];
    walkingEngineOutput.jointRequest.jointHardness.hardness[JointData::LAnklePitch] =
      theWalkingEngineParams.jointCalibration.legJointHardness[4];
    walkingEngineOutput.jointRequest.jointHardness.hardness[JointData::LAnkleRoll] =
      theWalkingEngineParams.jointCalibration.legJointHardness[5];
    walkingEngineOutput.jointRequest.jointHardness.hardness[JointData::RHipYawPitch] =
      theWalkingEngineParams.jointCalibration.legJointHardness[0];
    walkingEngineOutput.jointRequest.jointHardness.hardness[JointData::RHipRoll] =
      theWalkingEngineParams.jointCalibration.legJointHardness[1];
    walkingEngineOutput.jointRequest.jointHardness.hardness[JointData::RHipPitch] =
      theWalkingEngineParams.jointCalibration.legJointHardness[2];
    walkingEngineOutput.jointRequest.jointHardness.hardness[JointData::RKneePitch] =
      theWalkingEngineParams.jointCalibration.legJointHardness[3];
    walkingEngineOutput.jointRequest.jointHardness.hardness[JointData::RAnklePitch] =
      theWalkingEngineParams.jointCalibration.legJointHardness[4];
    walkingEngineOutput.jointRequest.jointHardness.hardness[JointData::RAnkleRoll] =
      theWalkingEngineParams.jointCalibration.legJointHardness[5];
    waitCounter--;
  }

  // set arm hardness
  for (int i = JointData::LShoulderPitch; i <= JointData::RElbowRoll; i++) {
    walkingEngineOutput.jointRequest.jointHardness.hardness[i] = HardnessData::useDefault;
  }

  for (int i = 0; i < 12; i++) {
    walkingEngineOutput.jointRequest.jointAngles.angles[JointData::LHipYawPitch + i] +=
      theWalkingEngineParams.jointCalibration.jointCalibration[i];
  }

  // TODO: replace this with other logging mechanisms
  // TODO: Care about your own problems
  LOG("GlobalAngles", "Joints LHipYawPitch", theJointData.angles[JointData::LHipYawPitch]);
  LOG("GlobalAngles", "Joints LHipRoll", theJointData.angles[JointData::LHipRoll]);
  LOG("GlobalAngles", "Joints LHipPitch", theJointData.angles[JointData::LHipPitch]);
  LOG("GlobalAngles", "Joints LKneePitch", theJointData.angles[JointData::LKneePitch]);
  LOG("GlobalAngles", "Joints LAnklePitch", theJointData.angles[JointData::LAnklePitch]);
  LOG("GlobalAngles", "Joints LAnkleRoll", theJointData.angles[JointData::LAnkleRoll]);

  LOG("GlobalAngles", "Joints RHipYawPitch", theJointData.angles[JointData::RHipYawPitch]);
  LOG("GlobalAngles", "Joints RHipRoll", theJointData.angles[JointData::RHipRoll]);
  LOG("GlobalAngles", "Joints RHipPitch", theJointData.angles[JointData::RHipPitch]);
  LOG("GlobalAngles", "Joints RKneePitch", theJointData.angles[JointData::RKneePitch]);
  LOG("GlobalAngles", "Joints RAnklePitch", theJointData.angles[JointData::RAnklePitch]);
  LOG("GlobalAngles", "Joints RAnkleRoll", theJointData.angles[JointData::RAnkleRoll]);

  LOG("GlobalAngles",
      "LimbCombinator LHipYawPitch",
      walkingEngineOutput.jointRequest.jointAngles.angles[JointData::LHipYawPitch]);
  LOG("GlobalAngles", "LimbCombinator LHipRoll", walkingEngineOutput.jointRequest.jointAngles.angles[JointData::LHipRoll]);
  LOG("GlobalAngles", "LimbCombinator LHipPitch", walkingEngineOutput.jointRequest.jointAngles.angles[JointData::LHipPitch]);
  LOG(
    "GlobalAngles", "LimbCombinator LKneePitch", walkingEngineOutput.jointRequest.jointAngles.angles[JointData::LKneePitch]);
  LOG("GlobalAngles",
      "LimbCombinator LAnklePitch",
      walkingEngineOutput.jointRequest.jointAngles.angles[JointData::LAnklePitch]);
  LOG(
    "GlobalAngles", "LimbCombinator LAnkleRoll", walkingEngineOutput.jointRequest.jointAngles.angles[JointData::LAnkleRoll]);

  LOG("GlobalAngles",
      "LimbCombinator RHipYawPitch",
      walkingEngineOutput.jointRequest.jointAngles.angles[JointData::RHipYawPitch]);
  LOG("GlobalAngles", "LimbCombinator RHipRoll", walkingEngineOutput.jointRequest.jointAngles.angles[JointData::RHipRoll]);
  LOG("GlobalAngles", "LimbCombinator RHipPitch", walkingEngineOutput.jointRequest.jointAngles.angles[JointData::RHipPitch]);
  LOG(
    "GlobalAngles", "LimbCombinator RKneePitch", walkingEngineOutput.jointRequest.jointAngles.angles[JointData::RKneePitch]);
  LOG("GlobalAngles",
      "LimbCombinator RAnklePitch",
      walkingEngineOutput.jointRequest.jointAngles.angles[JointData::RAnklePitch]);
  LOG(
    "GlobalAngles", "LimbCombinator RAnkleRoll", walkingEngineOutput.jointRequest.jointAngles.angles[JointData::RAnkleRoll]);

  LOG("GlobalAngles", "Walking Engine Time", walkingEngineTime);

  MARK("GlobalAngles", "Ego-CoM x");
  MARK("GlobalAngles", "Ego-CoM y");
  MARK("GlobalAngles", "Ego-CoM z");
  PLOT("module:LimbCombinator:Offset", angleoffset[JointData::RAnklePitch][12]);
  PLOT("module:LimbCombinator:Target", walkingEngineOutput.jointRequest.jointAngles.angles[JointData::RAnklePitch]);
  PLOT("module:LimbCombinator:Measured", theJointData.angles[JointData::RAnklePitch]);
}

MAKE_MODULE(LimbCombinator, dortmundWalkingEngine)
