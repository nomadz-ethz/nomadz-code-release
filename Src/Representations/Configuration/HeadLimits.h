/**
 * @file HeadLimits.h
 *
 * Declaration of a class for representing the limits of the head joints.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Felix Wenk
 */

#pragma once

#include "Core/Math/Vector2.h"
#include "Core/Math/Vector3.h"
#include "Core/Streams/AutoStreamable.h"

class RobotCameraMatrix;

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/head_limits.hpp"
#endif
STREAMABLE_DECLARE(HeadLimits)

STREAMABLE_ROS(HeadLimits, {
private:
  bool intersectionWithShoulderPlane(const RobotCameraMatrix& robotCameraMatrix,
                                     const Vector3<>& shoulderInOrigin,
                                     const float imageTilt,
                                     Vector3<>& intersection) const;

public:
  Vector2<> getTiltBound(float pan) const;

  /**
   * Method to determine whether the image would show mostly parts of the shoulder.
   * @param robotCameraMatrix Position and orientation of the camera in origin coordinates.
   * @param shoulderInOrigin Vector to the shouler in origin coordinates.
   * @param imageTilt 0 for center of image, <0 to move the intersection point upwards in the image, >0 to move it downwards.
   * @return true if the target point specified by imageTilt is hidden by the shoulder.
   */
  bool imageCenterHiddenByShoulder(const RobotCameraMatrix& robotCameraMatrix,
                                   const Vector3<>& shoulderInOrigin,
                                   const float imageTilt,
                                   const float hysteresis = 0.0f) const;

  /**
   * Calculates the upper intersection point of the vertical line through the center of the image
   * and the edge of the circle around the shoulder.
   * @param robotCameraMatrix Position and orientation of the camera in origin coordinates.
   * @param shoulderInOrigin Vector to the shouler in origin coordinates.
   * @param intersection The intersection point in origin coordinates (output parameter).
   * @return true if such an intersecion point exists.
   */
  bool intersectionWithShoulderEdge(
    const RobotCameraMatrix& robotCameraMatrix, const Vector3<>& shoulderInOrigin, Vector3<>& intersection) const;

  float maxPan() const {
    ASSERT(!intervals.empty());
    return intervals.back();
  }
  float minPan() const {
    ASSERT(!intervals.empty());
    return intervals.front();
  }

  /**< Draws this representation. */
  void draw() const, FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::HeadLimits::shoulder_radius, shoulderRadius),
    FIELD_WRAPPER_DEFAULT(std::vector<float>, nomadz_msgs::msg::HeadLimits::intervals, intervals),
    FIELD_WRAPPER_DEFAULT(std::vector<float>, nomadz_msgs::msg::HeadLimits::lower_bounds, lowerBounds),
    FIELD_WRAPPER_DEFAULT(std::vector<float>, nomadz_msgs::msg::HeadLimits::upper_bounds, upperBounds),
});
