/**
 * @file Odometer.h
 *
 * Some additional odometry information
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The originl author is <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 */

#pragma once

#include "Core/Math/Pose2D.h"
#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/odometer.hpp"
#endif
STREAMABLE_DECLARE(Odometer)

STREAMABLE_ROS(Odometer,
               {
                 ,
                 FIELD_WRAPPER(float,
                               10000.f,
                               nomadz_msgs::msg::Odometer::distance_walked,
                               distanceWalked), /** Total distance walked since start of B-Human software */
                 FIELD_WRAPPER_DEFAULT(Pose2D,
                                       nomadz_msgs::msg::Odometer::odometry_offset,
                                       odometryOffset), /** Odometry difference since last frame */
               });
