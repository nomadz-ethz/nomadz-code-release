/**
 * @file FieldPosition.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Function.h"
#include "Core/Math/Pose2D.h"
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/field_position.hpp"
#endif

STREAMABLE_DECLARE(FieldPosition)

STREAMABLE_ROS(
  FieldPosition,
  {
    ,
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::FieldPosition::ready_pose, readyPose), // Global Pose for ready position
    FIELD_WRAPPER_DEFAULT(Pose2D,
                          nomadz_msgs::msg::FieldPosition::stand_by_pose,
                          standByPose), // Global reference pose when away from action
    FIELD_WRAPPER_DEFAULT(Pose2D,
                          nomadz_msgs::msg::FieldPosition::search_pose,
                          searchPose), // Global reference pose for ball search
    FIELD_WRAPPER_DEFAULT(Pose2D,
                          nomadz_msgs::msg::FieldPosition::current_target_pose,
                          currentTargetPose), // Global reference pose when participate in the action
  });
