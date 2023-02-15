/**
 * @file PassHelper.h
 *
 * Declaration of the PassHelper representation.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Math/Vector2.h"

#include "Core/Streams/FieldWrapper.h"

STREAMABLE_DECLARE_LEGACY(PassHelper)
STREAMABLE_LEGACY(PassHelper, {
public:
  ENUM(TypePass, forward, lateral, backward, shot); // Enumeration that tell which kind of pass will be performed
  ,
    FIELD_WRAPPER_LEGACY(
      bool,
      false,
      nomadz_msgs::msg::PassHelper::found_pass_point,
      foundPassPoint), // Flag that tell if the algorithm found a pass point or not. If false -> Kick to opponents goal

    FIELD_WRAPPER_DEFAULT_LEGACY(
      Vector2<>, nomadz_msgs::msg::PassHelper::pass_coordinate, passCoordinate), // Coordinate of the pass

    FIELD_WRAPPER_LEGACY(float,
                         1000,
                         nomadz_msgs::msg::PassHelper::distance_pass,
                         distancePass), // Distance Ball Robot (needed for calculate the strength of the kick

    FIELD_WRAPPER_LEGACY(
      int, 0, nomadz_msgs::msg::PassHelper::id_reciever_pass, idRecieverPass), // Number of the receiving robot

    FIELD_WRAPPER_LEGACY(
      TypePass, shot, nomadz_msgs::msg::PassHelper::type_pass, typePass), // Type of pass that will be performed
});
