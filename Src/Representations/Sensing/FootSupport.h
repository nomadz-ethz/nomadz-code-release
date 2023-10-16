/**
 * @file FootSupport.h
 *
 * This file defines a representation that describes an abstract distribution of
 * how much each foot supports the weight of the robot. Positive value mean the
 * left foot supports more weight, while negative value mean the left foot supports
 * more weight.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Thomas Röfer
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Math/Pose2D.h"
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/leg.hpp"
#include "nomadz_msgs/msg/foot_support.hpp"
#endif
STREAMABLE_DECLARE(Leg)
STREAMABLE_DECLARE(FootSupport)

STREAMABLE_ROS(Leg, {
public:
  ENUM(Side, left, right);
  ENUM(StepType, normal, prepare, kick);
  , FIELD_WRAPPER(Side, left, nomadz_msgs::msg::Leg::side, side),
    FIELD_WRAPPER(float, 0.25f, nomadz_msgs::msg::Leg::step_duration, stepDuration),
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::Leg::leg_pose, legPose),
    FIELD_WRAPPER(StepType, normal, nomadz_msgs::msg::Leg::step_type, stepType),
});

STREAMABLE_ROS(
  FootSupport,
  {
    ,
    FIELD_WRAPPER(float,
                  0.f,
                  nomadz_msgs::msg::FootSupport::support,
                  support), /** Unitless distribution of the support over both feet (left - right). */
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::FootSupport::switched, switched), /** The support foot switched. */
    FIELD_WRAPPER(bool,
                  false,
                  nomadz_msgs::msg::FootSupport::predicted_switched,
                  predictedSwitched), /** The support foot switched but predicted 3 frames earlier. */
  });
