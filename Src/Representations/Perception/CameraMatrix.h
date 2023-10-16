/**
 * @file CameraMatrix.h
 *
 * Declaration of CameraMatrix and RobotCameraMatrix representation.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#pragma once

#include "Core/Math/Pose3D.h"
#include "Core/Streams/FieldWrapper.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/CameraCalibration.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/camera_matrix.hpp"
#endif
STREAMABLE_DECLARE_WITH_POSE3D(CameraMatrix)

/**
 * Matrix describing transformation from center of hip to camera.
 */
class RobotCameraMatrix : public Pose3D {
public:
  /** Draws the camera matrix. */
  void draw() const;

  void computeRobotCameraMatrix(const RobotDimensions& robotDimensions,
                                float headYaw,
                                float headPitch,
                                const CameraCalibration& cameraCalibration,
                                bool upperCamera);
  RobotCameraMatrix() {}
  RobotCameraMatrix(const RobotDimensions& robotDimensions,
                    const float headYaw,
                    const float headPitch,
                    const CameraCalibration& cameraCalibration,
                    bool upperCamera);
};

/**
 * Matrix describing transformation from ground (center between booth feet) to camera.
 */
STREAMABLE_WITH_POSE3D(CameraMatrix, {
public:
  /** Kind of copy-constructor.
   * @param pose The other pose.
   */
  CameraMatrix(const Pose3D& pose);

  void computeCameraMatrix(
    const Pose3D& torsoMatrix, const Pose3D& robotCameraMatrix, const CameraCalibration& cameraCalibration);
  CameraMatrix(const Pose3D& torsoMatrix, const Pose3D& robotCameraMatrix, const CameraCalibration& cameraCalibration);

  /** Draws the camera matrix. */
  void draw() const,
    FIELD_WRAPPER(
      bool, true, nomadz_msgs::msg::CameraMatrix::is_valid, isValid), /**< Matrix is only valid if motion was stable. */
});
