/**
 * @file InverseKinematic.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Alexander Härtl
 */

#pragma once

#include "Core/Math/Rotation.h"

#include "Core/Math/Common.h"
#include "Core/Math/Matrix.h"
#include "Core/Math/Vector.h"
#include "Core/Math/Vector2.h"
#include "Core/Math/Vector3.h"
#include "Core/Math/RotationMatrix.h"
#include <Eigen/Eigen>

class CameraCalibration;
class JointData;
class JointCalibration;
class Pose3D;
class RobotDimensions;

class InverseKinematic {
public:
  /**
   * This method calculates the joint angles for the legs of the robot from a Pose3D for each leg.
   * @param positionLeft The desired position (translation + rotation) of the left foots ankle point.
   * @param positionRight The desired position (translation + rotation) of the right foots ankle point.
   * @param jointAngles The instance of JointAngles where the resulting joint angles are written into.
   * @param robotDimensions The Robot Dimensions needed for calculation.
   * @param ratio The ratio between the left and right yaw angle.
   * @return Whether the target position was reachable or not (if the given target position is not reachable the computation
   * proceeds using the closest reachable position near the target).
   */
  static bool calcLegJoints(const Pose3D& positionLeft,
                            const Pose3D& positionRight,
                            JointData& jointAngles,
                            const RobotDimensions& robotDimensions,
                            float ratio = 0.5f);

  /**
   * This method calculates the joint angles for the legs of the robot from a Pose3D for each leg and the body ptch and roll.
   * @param positionLeft The desired position (translation + rotation) of the left foots point
   * @param positionRight The desired position (translation + rotation) of the right foots point
   * @param bodyRotation The rotation of the body around the x-Axis and y-Axis
   * @param jointAngles The instance of JointAngles where the resulting joint angles are written into.
   * @param robotDimensions The RobotDimensions needed for calculation
   * @param ratio The ratio between the left and right yaw angle
   * @return Whether the target position was reachable or not (if the given target position is not reachable the computation
   * proceeds using the closest reachable position near the target)
   */
  static bool calcLegJoints(const Pose3D& positionLeft,
                            const Pose3D& positionRight,
                            const Vector2<>& bodyRotation,
                            JointData& jointAngles,
                            const RobotDimensions& robotDimensions,
                            float ratio = 0.5f);
  static bool calcLegJoints(const Pose3D& positionLeft,
                            const Pose3D& positionRight,
                            const Eigen::Quaternionf& bodyRotation,
                            JointData& jointAngles,
                            const RobotDimensions& robotDimensions,
                            float ratio = 0.5f);

  /**
   * Solves the inverse kinematics for the head of the Nao such that the camera looks at a certain point.
   * @param position Point the camera should look at in cartesian space relative to the robot origin.
   * @param imageTilt Tilt angle at which the point should appear in the image (pi/2: center of image, less than pi/2 =>
   * closer to the top of the image.)
   * @param panTilt Vector [pan, tilt] containing the resulting joint angles.
   * @param lowerCamera true if the joint angles are to be determined for the lower camera, false for the upper camera.
   * @param robotDimensions The robot dimensions needed for the calculation.
   * @param cameraCalibration The camera calibration
   */
  static void calcHeadJoints(const Vector3<>& position,
                             const float imageTilt,
                             const RobotDimensions& robotDimensions,
                             const bool lowerCamera,
                             Vector2<>& panTilt,
                             const CameraCalibration& cameraCalibration);

  static RotationMatrix eigenQuatToRotmat(Eigen::Quaternionf quat);
}; // namespace InverseKinematic
