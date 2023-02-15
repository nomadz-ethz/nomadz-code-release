/**
 * @file JointCalibrator.cpp
 *
 * This file implements a module with tools for calibrating leg joints.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#include "JointCalibrator.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Representations/Sensing/RobotModel.h"

MAKE_MODULE(JointCalibrator, Motion Infrastructure)

void JointCalibrator::update(JointCalibration& jointCalibration) {
  bool allActive = true;
  for (int i = JointData::LHipYawPitch; i <= JointData::RAnkleRoll; ++i) {
    allActive &= theJointRequest.jointAngles.angles[i] != JointData::off;
  }

  DEBUG_RESPONSE_ONCE("module:JointCalibrator:reset", {
    Global::getDebugRequestTable().disable("module:JointCalibrator:reset");
    for (int i = JointData::LHipYawPitch; i <= JointData::RAnkleRoll; ++i) {
      jointCalibration.joints[i].offset = 0;
    }

    offsets.clear();
    lastOffsets.clear();
  });

  // This automatically calibrate all joints except for the two hip pitch joints
  DEBUG_RESPONSE_ONCE("module:JointCalibrator:sitPosReset", {
    Global::getDebugRequestTable().disable("module:JointCalibrator:sitPosReset");
    for (int i = 0; i <= JointData::RAnkleRoll; ++i) {
      jointCalibration.joints[i].offset = theJointData.angles[i] - theJointRequest.jointAngles.angles[i];
    }
  });

  DEBUG_RESPONSE_ONCE("module:JointCalibrator:standPosReset", {
    Global::getDebugRequestTable().disable("module:JointCalibrator:standPosReset");
    for (int i = JointData::LHipYawPitch; i <= JointData::RAnkleRoll; ++i) {
      jointCalibration.joints[i].offset = 0;
    }
    offsets.clear();
    lastOffsets.clear();

    for (int i = 0; i <= JointData::RAnkleRoll; ++i) {
      jointCalibration.joints[i].offset = theJointData.angles[i] - theJointRequest.jointAngles.angles[i];
    }
  });

  DEBUG_RESPONSE("module:JointCalibrator:captureJointPlay", {
    for (int i = JointData::LHipYawPitch; i <= JointData::RAnkleRoll; ++i) {
      jointPlayMin[i] = std::min(jointPlayMin[i], theJointData.angles[i] - theJointRequest.jointAngles.angles[i]);
      jointPlayMax[i] = std::max(jointPlayMax[i], theJointData.angles[i] - theJointRequest.jointAngles.angles[i]);
      jointCalibration.joints[i].jointPlay = jointPlayMax[i] - jointPlayMin[i];
    }
  });

  DEBUG_RESPONSE_ONCE("module:JointCalibrator:capture", {
    Global::getDebugRequestTable().disable("module:JointCalibrator:capture");

    if (!allActive)
      OUTPUT(idText, text, "Error: capturing not possible because at least one joint is off");
    else
      for (int i = JointData::LHipYawPitch; i <= JointData::RAnkleRoll; ++i) {
        jointCalibration.joints[i].offset += theJointData.angles[i] - theJointRequest.jointAngles.angles[i];
      }
  });

  MODIFY("module:JointCalibrator:offsets", offsets);
  if (offsets != lastOffsets) {
    if (!allActive) {
      OUTPUT(idText, text, "Error: setting offsets not possible because at least one joint is off");
    } else {
      Offsets additionalOffsets = offsets;
      additionalOffsets -= lastOffsets;
      Pose3D rotationOffset(RotationMatrix(additionalOffsets.bodyRotation));
      Pose3D leftOffset(RotationMatrix(additionalOffsets.leftFoot.rotation), additionalOffsets.leftFoot.translation);
      Pose3D rightOffset(RotationMatrix(additionalOffsets.rightFoot.rotation), additionalOffsets.rightFoot.translation);
      RobotModel robotModel(theJointRequest.jointAngles, theRobotDimensions, theMassCalibration);
      JointData jointData;
      InverseKinematic::calcLegJoints(
        Pose3D(rotationOffset).conc(robotModel.limbs[MassCalibration::footLeft]).conc(leftOffset),
        Pose3D(rotationOffset).conc(robotModel.limbs[MassCalibration::footRight]).conc(rightOffset),
        jointData,
        theRobotDimensions);

      for (int i = JointData::LHipYawPitch; i <= JointData::RAnkleRoll; ++i) {
        jointCalibration.joints[i].offset += jointData.angles[i] - theJointRequest.jointAngles.angles[i];
      }
    }

    lastOffsets = offsets;
  }
}
