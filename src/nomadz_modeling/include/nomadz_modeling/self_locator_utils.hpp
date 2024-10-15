#pragma once

#include <Eigen/Core>

#include "nomadz_configuration/field_dimensions.hpp"
#include "nomadz_core/geometry/pose.hpp"

namespace nomadz_modeling {

  /**
   *   @brief Implements planar rigid body dinamics given an offset in the robot frame
   *
   *   @param state The current state of the object
   *   @param velocity The velocity acting on the object
   */
  void dynamicsModel(Eigen::Vector3f& state, const Eigen::Vector3f& odometry_offset);

  /**
   *   @brief Implements landmark measurement model
   *
   *   @param state The current state of the object
   *   @param model The position of the original landmark in the world frame
   */
  Eigen::Vector2f landmarkMeasurementModel(const Eigen::Vector3f& state, const Eigen::Vector2f& model);

  /**
   *   @brief Implements robot pose measurement model
   *
   *   @param state The current state of the object
   */
  Eigen::Vector3f robotPoseMeasurementModel(const Eigen::Vector3f& state);

  /**
   *   @brief Implements line measurement model
   *
   *   @param state The current state of the object
   *   @param vertical If the perceived line is vertical or not
   */
  Eigen::Vector2f lineMeasurementModel(const Eigen::Vector3f& state, bool vertical);

  /**
   *   @brief Return starting position on field of a player given its player_number, according to the following rule:
   *
   *   /               7 /
   *   / 6               /
   *   /               5 /
   *   / 4               /
   *   /               3 /
   *   / 2               /
   *   /               1 /
   *   /      goal       /
   *
   *   @param player_number Player number
   *   @param half_field_length Length of half field
   *   @param half_field_width Width of half field
   */
  Eigen::Vector3f computeInitialState(int player_number, float half_field_length, float half_field_width);

} // namespace nomadz_modeling
