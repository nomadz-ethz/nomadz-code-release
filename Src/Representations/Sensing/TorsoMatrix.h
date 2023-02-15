/**
 * @file TorsoMatrix.h
 *
 * Declaration of class TorsoMatrix.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#pragma once

#include "Core/Math/Pose3D.h"
#include "Core/Streams/AutoStreamable.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/torso_matrix.hpp"
#endif
STREAMABLE_DECLARE_WITH_POSE3D(TorsoMatrix)

/**
 * @class TorsoMatrix
 * Matrix describing the transformation from ground to the robot torso.
 */
STREAMABLE_WITH_POSE3D(TorsoMatrix,
                       {
                         ,
                         FIELD_WRAPPER_DEFAULT(Pose3D, nomadz_msgs::msg::TorsoMatrix::offset, offset),
                         FIELD_WRAPPER(bool, false, nomadz_msgs::msg::TorsoMatrix::is_valid, isValid),
                         FIELD_WRAPPER(bool, false, nomadz_msgs::msg::TorsoMatrix::left_support_foot, leftSupportFoot),
                       });
