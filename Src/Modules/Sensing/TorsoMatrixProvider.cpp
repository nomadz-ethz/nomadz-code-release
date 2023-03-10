/**
 * @file TorsoMatrixProvider.cpp
 *
 * Implementation of module TorsoMatrixProvider.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#include "TorsoMatrixProvider.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Debugging/DebugDrawings3D.h"

MAKE_MODULE(TorsoMatrixProvider, Sensing)

void TorsoMatrixProvider::update(TorsoMatrix& torsoMatrix) {
  // remove the z-rotation from the orientation data rotation matrix
  // TODO[flip] Logic was updated, verify this works as intended
  RotationMatrix torsoRotation(Vector3<>(
    theFilteredSensorData.data[FilteredSensorData::angleX], theFilteredSensorData.data[FilteredSensorData::angleY], 0.0));

  // calculate "center of hip" position from left foot
  Pose3D fromLeftFoot(torsoRotation);
  fromLeftFoot.conc(theRobotModel.limbs[MassCalibration::footLeft]);
  fromLeftFoot.translate(0, 0, -theRobotDimensions.heightLeg5Joint);
  fromLeftFoot.translation *= -1.;
  fromLeftFoot.rotation = torsoRotation;

  // calculate "center of hip" position from right foot
  Pose3D fromRightFoot(torsoRotation);
  fromRightFoot.conc(theRobotModel.limbs[MassCalibration::footRight]);
  fromRightFoot.translate(0, 0, -theRobotDimensions.heightLeg5Joint);
  fromRightFoot.translation *= -1.;
  fromRightFoot.rotation = torsoRotation;

  // get foot z-rotations
  const Pose3D& leftFootInverse(theRobotModel.limbs[MassCalibration::footLeft].invert());
  const Pose3D& rightFootInverse(theRobotModel.limbs[MassCalibration::footRight].invert());
  const float leftFootZRotation = leftFootInverse.rotation.getZAngle();
  const float rightFootZRotation = rightFootInverse.rotation.getZAngle();

  // determine used foot
  const bool useLeft = fromLeftFoot.translation.z > fromRightFoot.translation.z;
  torsoMatrix.leftSupportFoot = useLeft;

  // calculate foot span
  const Vector3<> newFootSpan(fromRightFoot.translation - fromLeftFoot.translation);

  // and construct the matrix
  Pose3D newTorsoMatrix;
  newTorsoMatrix.translate(newFootSpan.x / (useLeft ? 2.f : -2.f), newFootSpan.y / (useLeft ? 2.f : -2.f), 0);
  newTorsoMatrix.conc(useLeft ? fromLeftFoot : fromRightFoot);

  // calculate torso offset
  if (torsoMatrix.translation.z != 0) // the last torso matrix should be valid
  {
    Pose3D& torsoOffset(torsoMatrix.offset);
    torsoOffset = torsoMatrix.invert();
    torsoOffset.translate(lastFootSpan.x / (useLeft ? 2.f : -2.f), lastFootSpan.y / (useLeft ? 2.f : -2.f), 0);
    torsoOffset.rotateZ(useLeft ? float(leftFootZRotation - lastLeftFootZRotation)
                                : float(rightFootZRotation - lastRightFootZRotation));
    torsoOffset.translate(newFootSpan.x / (useLeft ? -2.f : 2.f), newFootSpan.y / (useLeft ? -2.f : 2.f), 0);
    torsoOffset.conc(newTorsoMatrix);
  }

  // adopt new matrix and footSpan
  (Pose3D&)torsoMatrix = newTorsoMatrix;
  lastLeftFootZRotation = leftFootZRotation;
  lastRightFootZRotation = rightFootZRotation;
  lastFootSpan = newFootSpan;

  // valid?
  torsoMatrix.isValid = theGroundContactState.contact;
  DECLARE_DEBUG_DRAWING3D("module:TorsoMatrixProvider:torsoMatrix", "field");
  COMPLEX_DRAWING3D("module:TorsoMatrixProvider:torsoMatrix", {
    float vectorWidth = 1.5f;
    const float vectorLen = 30.0f;
    const Vector3<>& robotPose = Vector3<>(theRobotPose.translation.x, theRobotPose.translation.y, 0.0f);
    RotationMatrix robotPoseCorrection = RotationMatrix().rotateZ(theRobotPose.rotation);
    const Vector3<>& origin = robotPose + robotPoseCorrection * torsoMatrix.translation;
    const Vector3<>& origin_x = origin + robotPoseCorrection * torsoMatrix.rotation.c0 * vectorLen;
    const Vector3<>& origin_y = origin + robotPoseCorrection * torsoMatrix.rotation.c1 * vectorLen;
    const Vector3<>& origin_z = origin + robotPoseCorrection * torsoMatrix.rotation.c2 * vectorLen;
    SPHERE3D("module:TorsoMatrixProvider:torsoMatrix",
             robotPose.x,
             robotPose.y,
             robotPose.z,
             vectorWidth * 2,
             ColorRGBA(255, 0, 0));

    LINE3D("module:TorsoMatrixProvider:torsoMatrix",
           robotPose.x,
           robotPose.y,
           robotPose.z,
           origin.x,
           origin.y,
           origin.z,
           vectorWidth * 3,
           ColorRGBA(255, 0, 0));
    LINE3D("module:TorsoMatrixProvider:torsoMatrix",
           origin.x,
           origin.y,
           origin.z,
           origin_x.x,
           origin_x.y,
           origin_x.z,
           vectorWidth * 3,
           ColorRGBA(255, 0, 0));
    LINE3D("module:TorsoMatrixProvider:torsoMatrix",
           origin.x,
           origin.y,
           origin.z,
           origin_y.x,
           origin_y.y,
           origin_y.z,
           vectorWidth * 3,
           ColorRGBA(0, 255, 0));
    LINE3D("module:TorsoMatrixProvider:torsoMatrix",
           origin.x,
           origin.y,
           origin.z,
           origin_z.x,
           origin_z.y,
           origin_z.z,
           vectorWidth * 3,
           ColorRGBA(0, 0, 255));
  });
  //
  PLOT("module:TorsoMatrixProvider:footSpanX", newFootSpan.x);
  PLOT("module:TorsoMatrixProvider:footSpanY", newFootSpan.y);
  PLOT("module:TorsoMatrixProvider:footSpanZ", newFootSpan.z);

  PLOT("module:TorsoMatrixProvider:torsoMatrixX", torsoMatrix.translation.x);
  PLOT("module:TorsoMatrixProvider:torsoMatrixY", torsoMatrix.translation.y);
  PLOT("module:TorsoMatrixProvider:torsoMatrixZ", torsoMatrix.translation.z);
}

/*
void TorsoMatrixProvider::update(FilteredOdometryOffset& odometryOffset)
{
  Pose2D odometryOffset;

  if(lastTorsoMatrix.translation.z != 0.)
  {
    Pose3D odometryOffset3D(lastTorsoMatrix);
    odometryOffset3D.conc(theTorsoMatrix.offset);
    odometryOffset3D.conc(theTorsoMatrix.invert());

    odometryOffset.translation.x = odometryOffset3D.translation.x;
    odometryOffset.translation.y = odometryOffset3D.translation.y;
    odometryOffset.rotation = odometryOffset3D.rotation.getZAngle();
  }

  PLOT("module:TorsoMatrixProvider:odometryOffsetX", odometryOffset.translation.x);
  PLOT("module:TorsoMatrixProvider:odometryOffsetY", odometryOffset.translation.y);
  PLOT("module:TorsoMatrixProvider:odometryOffsetRotation", toDegrees(odometryOffset.rotation));

  (Pose3D&)lastTorsoMatrix = theTorsoMatrix;
}
*/
void TorsoMatrixProvider::update(OdometryData& odometryData) {
  Pose2D odometryOffset;

  if (lastTorsoMatrix.translation.z != 0.) {
    Pose3D odometryOffset3D(lastTorsoMatrix);
    odometryOffset3D.conc(theTorsoMatrix.offset);
    odometryOffset3D.conc(theTorsoMatrix.invert());

    odometryOffset.translation.x = odometryOffset3D.translation.x;
    odometryOffset.translation.y = odometryOffset3D.translation.y;
    odometryOffset.rotation = odometryOffset3D.rotation.getZAngle();
  }

  PLOT("module:TorsoMatrixProvider:odometryOffsetX", odometryOffset.translation.x);
  PLOT("module:TorsoMatrixProvider:odometryOffsetY", odometryOffset.translation.y);
  PLOT("module:TorsoMatrixProvider:odometryOffsetRotation", toDegrees(odometryOffset.rotation));

  odometryData += odometryOffset;

  (Pose3D&)lastTorsoMatrix = theTorsoMatrix;
}
