/**
 * @file RobotCameraMatrixProvider.h
 *
 * This file declares a class to calculate the position of the camera relative to the body for the Nao.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"

MODULE(RobotCameraMatrixProvider)
REQUIRES(CameraCalibration)
REQUIRES(RobotDimensions)
REQUIRES(FilteredJointData)
REQUIRES(CameraInfo)
PROVIDES_WITH_MODIFY_AND_DRAW(RobotCameraMatrix);
END_MODULE

class RobotCameraMatrixProvider : public RobotCameraMatrixProviderBase {
private:
  void update(RobotCameraMatrix& robotCameraMatrix);
};
