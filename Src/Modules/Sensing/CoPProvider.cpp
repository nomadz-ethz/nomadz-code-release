/**
 * @file CoPProvider.cpp
 *
 * This file implements a module that provides information about the current ZMP.
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#include "CoPProvider.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Tools/DortmundWalkingEngine/StepData.h"

void CoPProvider::update(ZMPModel& zmpModel) {
  float leftTotal, rightTotal;
#if 0
	Vector3<> lCoP_FCS(theFilteredSensorData.data[SensorData::lCoPX], theFilteredSensorData.data[SensorData::lCoPY], 0);
	Vector3<> rCoP_FCS(theFilteredSensorData.data[SensorData::rCoPX], theFilteredSensorData.data[SensorData::rCoPY], 0);
#else
  Vector3<> lCoP_FCS = Vector3<>();
  Vector3<> rCoP_FCS = Vector3<>();

  //  float leftSum = theFsrSensorData.left[FsrSensorData::fl] +
  //    theFsrSensorData.left[FsrSensorData::fr] +
  //    theFsrSensorData.left[FsrSensorData::bl] +
  //    theFsrSensorData.left[FsrSensorData::br];

  float leftSum = theSensorData.data[SensorData::fsrLFL] + theSensorData.data[SensorData::fsrLFR] +
                  theSensorData.data[SensorData::fsrLBL] + theSensorData.data[SensorData::fsrLBR];

  //  if (leftSum > 0)
  //  {
  //    lCoP_FCS.x = (theFsrSensorData.left[FsrSensorData::fl] * 0.07025f +
  //      theFsrSensorData.left[FsrSensorData::fr] * 0.07025f +
  //      theFsrSensorData.left[FsrSensorData::bl] * -0.03025f +
  //      theFsrSensorData.left[FsrSensorData::br] * -0.02965f) / leftSum;
  //    lCoP_FCS.y = (theFsrSensorData.left[FsrSensorData::fl] * 0.0299f +
  //      theFsrSensorData.left[FsrSensorData::fr] * -0.0231f +
  //      theFsrSensorData.left[FsrSensorData::bl] * 0.0299f +
  //      theFsrSensorData.left[FsrSensorData::br] * -0.0191f) / leftSum;
  //  }

  if (leftSum > 0) {
    lCoP_FCS.x = (theSensorData.data[SensorData::fsrLFL] * 0.07025f + theSensorData.data[SensorData::fsrLFR] * 0.07025f +
                  theSensorData.data[SensorData::fsrLBL] * -0.03025f + theSensorData.data[SensorData::fsrLBR] * -0.02965f) /
                 leftSum;
    lCoP_FCS.y = (theSensorData.data[SensorData::fsrLFL] * 0.0299f + theSensorData.data[SensorData::fsrLFR] * -0.0231f +
                  theSensorData.data[SensorData::fsrLBL] * 0.0299f + theSensorData.data[SensorData::fsrLBR] * -0.0191f) /
                 leftSum;
  }

  //  float rightSum = theFsrSensorData.right[FsrSensorData::fl] +
  //    theFsrSensorData.right[FsrSensorData::fr] +
  //    theFsrSensorData.right[FsrSensorData::bl] +
  //    theFsrSensorData.right[FsrSensorData::br];
  //
  //  if (rightSum > 0)
  //  {
  //    rCoP_FCS.x() = (theFsrSensorData.right[FsrSensorData::fl] * 0.07025f +
  //      theFsrSensorData.right[FsrSensorData::fr] * 0.07025f +
  //      theFsrSensorData.right[FsrSensorData::bl] * -0.03025f +
  //      theFsrSensorData.right[FsrSensorData::br] * -0.02965f) / rightSum;
  //    rCoP_FCS.y() = (theFsrSensorData.right[FsrSensorData::fl] * 0.0231f +
  //      theFsrSensorData.right[FsrSensorData::fr] * -0.0299f +
  //      theFsrSensorData.right[FsrSensorData::bl] * 0.0191f +
  //      theFsrSensorData.right[FsrSensorData::br] * -0.0299f) / rightSum;
  //  }

  float rightSum = theSensorData.data[SensorData::fsrRFL] + theSensorData.data[SensorData::fsrRFR] +
                   theSensorData.data[SensorData::fsrRBL] + theSensorData.data[SensorData::fsrRBR];

  if (rightSum > 0) {
    rCoP_FCS.x = (theSensorData.data[SensorData::fsrRFL] * 0.07025f + theSensorData.data[SensorData::fsrRFR] * 0.07025f +
                  theSensorData.data[SensorData::fsrRBL] * -0.03025f + theSensorData.data[SensorData::fsrRBR] * -0.02965f) /
                 rightSum;
    rCoP_FCS.y = (theSensorData.data[SensorData::fsrRFL] * 0.0231f + theSensorData.data[SensorData::fsrRFR] * -0.0299f +
                  theSensorData.data[SensorData::fsrRBL] * 0.0191f + theSensorData.data[SensorData::fsrRBR] * -0.0299f) /
                 rightSum;
  }

#endif
// convert to robot coordinate system
#if 1
  Pose3D footLeft(theRobotModel.limbs[Limbs::footLeft].rotation, (theRobotModel.limbs[Limbs::footLeft].translation / 1000));
  Pose3D footRight(theRobotModel.limbs[Limbs::footRight].rotation,
                   (theRobotModel.limbs[Limbs::footRight].translation / 1000));
#else
  Pose3D footLeft(RotationMatrix_D(theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].r,
                                   theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].ry,
                                   theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].rx),
                  Vector3_D<double>(theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].x / 1000,
                                    theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].y / 1000,
                                    theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].z / 1000));
  Pose3D_D footRight(RotationMatrix_D(theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].r,
                                      theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].ry,
                                      theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].rx),
                     Vector3_D<double>(theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].x / 1000,
                                       theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].y / 1000,
                                       theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].z / 1000));
#endif

  Vector3<> lCoP_RCS = footLeft * lCoP_FCS;
  Vector3<> rCoP_RCS = footRight * rCoP_FCS;

  leftTotal = leftSum;
  rightTotal = rightSum;

  float robotMass = leftTotal + rightTotal;

  float lfac = 0.5f, rfac = 0.5f;

  if (robotMass > 0.001) {
    lfac = leftTotal / robotMass;
    rfac = rightTotal / robotMass;
  }

  zmpModel.zmp_acc = lCoP_RCS * lfac + rCoP_RCS * rfac;

  Pose3D footLeft_WCS(theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].x,
                      theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].y,
                      theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].z);
  footLeft_WCS.rotation = RotationMatrix(0.f, 0.f, theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].r);
  Pose3D footRight_WCS(theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].x,
                       theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].y,
                       theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].z);
  footRight_WCS.rotation = RotationMatrix(0.f, 0.f, theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].r);

  Vector3<> lCoP_WCS = footLeft_WCS * lCoP_FCS;
  Vector3<> rCoP_WCS = footRight_WCS * rCoP_FCS;

  zmpModel.ZMP_WCS = lCoP_WCS * lfac + rCoP_WCS * rfac;

  PLOT("module:CoPProvider:CoPx", zmpModel.zmp_acc.x);
  PLOT("module:CoPProvider:CoPy", zmpModel.zmp_acc.y);

  PLOT("module:CoPProvider:CoPxWCS", zmpModel.ZMP_WCS.x);
  PLOT("module:CoPProvider:CoPyWCS", zmpModel.ZMP_WCS.y);

  PLOT("module:CoPProvider:lCoP_FCS x", lCoP_FCS.x);
  PLOT("module:CoPProvider:lCoP_FCS y", lCoP_FCS.y);

  PLOT("module:CoPProvider:rCoP_FCS x", rCoP_FCS.x);
  PLOT("module:CoPProvider:rCoP_FCS y", rCoP_FCS.y);

  // PLOT("module:CoPProvider:lWeight", theFsrSensorData.leftTotal);
  // PLOT("module:CoPProvider:rWeight", theFsrSensorData.rightTotal);
}

MAKE_MODULE(CoPProvider, Sensing)
