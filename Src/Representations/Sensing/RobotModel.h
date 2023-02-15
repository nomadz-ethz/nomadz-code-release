/**
 * @file RobotModel.h
 *
 * Declaration of class RobotModel
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
 */

#pragma once

#include "Core/Math/Pose3D.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Tools/RobotParts/Limbs.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/robot_model.hpp"
#endif
STREAMABLE_DECLARE(RobotModel)

/**
 * @class RobotModel
 *
 * Contains information about extremities.
 */
STREAMABLE_ROS(RobotModel, {
public:
  /**
   * Constructs the RobotModel from given joint data.
   * @param joints The joint data.
   * @param robotDimensions The dimensions of the robot.
   * @param massCalibration The mass calibration of the robot.
   */
  RobotModel(const JointData& joints, const RobotDimensions& robotDimensions, const MassCalibration& massCalibration);

  /**
   * Recalculates the RobotModel from given joint data.
   * @param joints The joint data.
   * @param robotDimensions The dimensions of the robot.
   * @param massCalibration The mass calibration of the robot.
   */
  void setJointData(const JointData& joints, const RobotDimensions& robotDimensions, const MassCalibration& massCalibration);

  /**
   * Re-calculate the center of mass in this model.
   * @param massCalibration The mass calibration of the robot.
   */
  void updateCenterOfMass(const MassCalibration& massCalibration);

  /** Creates a 3-D drawing of the robot model. */
  void draw() const,
    FIELD_WRAPPER_DEFAULT(Pose3D[MassCalibration::numOfLimbs],
                          nomadz_msgs::msg::RobotModel::limbs,
                          limbs), /**< Coordinate frame of the limbs of the robot relative to the robot's origin. */
    FIELD_WRAPPER_DEFAULT(
      Vector3<>,
      nomadz_msgs::msg::RobotModel::center_of_mass,
      centerOfMass), /**< Position of the center of mass (center of gravity) relative to the robot's origin. */
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::RobotModel::total_mass, totalMass), /**< The mass of the robot. */
    FIELD_WRAPPER_DEFAULT(Pose3D, nomadz_msgs::msg::RobotModel::sole_left, soleLeft),
    FIELD_WRAPPER_DEFAULT(Pose3D, nomadz_msgs::msg::RobotModel::sole_right, soleRight),
});
