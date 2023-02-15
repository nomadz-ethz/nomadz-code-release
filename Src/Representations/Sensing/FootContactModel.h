/**
 * @file FootContactModel.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/foot_contact_model.hpp"
#endif
STREAMABLE_DECLARE(FootContactModel)

STREAMABLE_ROS(
  FootContactModel,
  {
    ,
    FIELD_WRAPPER(bool,
                  false,
                  nomadz_msgs::msg::FootContactModel::contact_left,
                  contactLeft), /**< do we have foot contact with the left foot? */
    FIELD_WRAPPER(bool,
                  false,
                  nomadz_msgs::msg::FootContactModel::contact_right,
                  contactRight), /**< do we have foot contact with the right foot? */
    FIELD_WRAPPER(int,
                  0,
                  nomadz_msgs::msg::FootContactModel::contact_duration_left,
                  contactDurationLeft), /**< duration (in frames) of the current contact of the left foot. 0 if no contact */
    FIELD_WRAPPER(
      int,
      0,
      nomadz_msgs::msg::FootContactModel::contact_duration_right,
      contactDurationRight), /**< duration (in frames) of the current contact of the right foot. 0 if no contact */
    FIELD_WRAPPER(unsigned,
                  0,
                  nomadz_msgs::msg::FootContactModel::last_contact_left,
                  lastContactLeft), /**< timestamp of the last contact detection of the left foot */
    FIELD_WRAPPER(unsigned,
                  0,
                  nomadz_msgs::msg::FootContactModel::last_contact_right,
                  lastContactRight), /**< timestamp of the last contact detection of the right foot */
  });
