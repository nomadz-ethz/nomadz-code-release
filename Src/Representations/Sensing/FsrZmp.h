/**
 * @file FsrZmp.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#pragma once

#include "Core/Math/Vector2.h"
#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

STREAMABLE_DECLARE_LEGACY(FsrZmp)

STREAMABLE_LEGACY(
  FsrZmp,
  {
    ,
    FIELD_WRAPPER_DEFAULT_LEGACY(Vector3<float>,
                                 nomadz_msgs::msg::FsrZmp::cop,
                                 cop), /**< center of pressure calculated over both feet relativ to robot origin */
    FIELD_WRAPPER_DEFAULT_LEGACY(
      Vector2<float>,
      nomadz_msgs::msg::FsrZmp::cop_left,
      copLeft), /**< center of pressure of left foot relative to left ankle without z coordinate */
    FIELD_WRAPPER_DEFAULT_LEGACY(
      Vector2<float>,
      nomadz_msgs::msg::FsrZmp::cop_right,
      copRight), /**< center of pressure of right foot relative to right ankle without z coordinate */
    FIELD_WRAPPER_LEGACY(float,
                         0.f,
                         nomadz_msgs::msg::FsrZmp::pressure,
                         pressure), /**< total pressure on all sensors (cannot be zero, at least FLT_EPSILON) */
    FIELD_WRAPPER_LEGACY(float,
                         0.f,
                         nomadz_msgs::msg::FsrZmp::pressure_left,
                         pressureLeft), /**< total pressure on the left foot  (cannot be zero, at least FLT_EPSILON)*/
    FIELD_WRAPPER_LEGACY(float,
                         0.f,
                         nomadz_msgs::msg::FsrZmp::pressure_right,
                         pressureRight), /**< total pressure on the right foot (cannot be zero, at least FLT_EPSILON)*/
  });
