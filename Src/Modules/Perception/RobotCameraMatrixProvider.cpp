/**
 * @file RobotCameraMatrixProvider.cpp
 *
 * This file implements a class to calculate the position of the camera relative to the body for the Nao.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
 * and Colin Graf
 */

#include "RobotCameraMatrixProvider.h"

MAKE_MODULE(RobotCameraMatrixProvider, Perception);

void RobotCameraMatrixProvider::update(RobotCameraMatrix& robotCameraMatrix) {
  robotCameraMatrix.computeRobotCameraMatrix(theRobotDimensions,
                                             theFilteredJointData.angles[JointData::HeadYaw],
                                             // TODO[flip] Verify this does not have to be inverted
                                             theFilteredJointData.angles[JointData::HeadPitch],
                                             theCameraCalibration,
                                             theCameraInfo.camera == CameraInfo::upper);
}
