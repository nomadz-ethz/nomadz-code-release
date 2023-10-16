/**
 * @file OrientationData.h
 *
 * Declaration of class OrientationData.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#pragma once

#include "Core/Math/RotationMatrix.h"
#include "Core/Streams/AutoStreamable.h"

/**
 * @class OrientationData
 * Encapsulates the orientation and velocity of the torso.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/orientation_data.hpp"
#endif
STREAMABLE_DECLARE(OrientationData)

STREAMABLE_ROS(
  OrientationData,
  {
    ,
    FIELD_WRAPPER_DEFAULT(RotationMatrix,
                          nomadz_msgs::msg::OrientationData::rotation,
                          rotation), /**< The rotation of the torso. */
    FIELD_WRAPPER_DEFAULT(Vector3<>,
                          nomadz_msgs::msg::OrientationData::velocity,
                          velocity), /**< The velocity along the x-, y- and z-axis relative to the torso. (in m/s) */
  });

STREAMABLE_ALIAS(GroundTruthOrientationData, OrientationData, {});
