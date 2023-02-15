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
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Thomas Röfer
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/foot_support.hpp"
#endif
STREAMABLE_DECLARE(FootSupport)
namespace Legs {
  ENUM(Leg, left, right);
}

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
