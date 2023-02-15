/**
 * @file PersonalData.h
 *
 * Declaration of a class representing information about the Robot.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once
#include "Core/Streams/AutoStreamable.h"
#include "Representations/Modeling/BallAfterKickPose.h"
#include "Core/Enum.h"
#include "Core/Math/Vector2.h"
#include <string.h>

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/personal_data.hpp"
#endif
STREAMABLE_DECLARE(PersonalData)

STREAMABLE_ROS(PersonalData, {
  public :,
  // FIXME: legacy code

  FIELD_WRAPPER(bool, false, nomadz_msgs::msg::PersonalData::reset, reset), /**lets Supporter know when to reset passTarget*/
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::PersonalData::possible_pass_point_chosen,
                        possiblePassPointChosen), /** possiblePassPoint chosen by the Supporter, global coordinates*/
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::PersonalData::pass_coordinate,
                        passCoordinate), /**Coordinate, where Ball is kicked to, determined by striker, such that
          Supporter knows more accurate position*/
  FIELD_WRAPPER(int, 0, nomadz_msgs::msg::PersonalData::num_of_sup, numOfSup),
  FIELD_WRAPPER(int, 0, nomadz_msgs::msg::PersonalData::num_of_str, numOfStr),
  FIELD_WRAPPER(float, 10000, nomadz_msgs::msg::PersonalData::min_distance_to_ball, minDistanceToBall),
  FIELD_WRAPPER(int, 0, nomadz_msgs::msg::PersonalData::my_distance_to_ball, myDistanceToBall),

  ////// For PassHelper (Filippo Martinoni)

  FIELD_WRAPPER(bool, false, nomadz_msgs::msg::PersonalData::need_to_calculate_pass, needToCalculatePass),
  FIELD_WRAPPER(
    bool,
    false,
    nomadz_msgs::msg::PersonalData::found_pass_point,
    foundPassPoint), // Flag that tell if the algorithm found a pass point or not. If false -> Kick to opponents goal

  FIELD_WRAPPER(float,
                1000,
                nomadz_msgs::msg::PersonalData::distance_pass,
                distancePass), // Distance Ball Robot (needed for calculate the strength of the kick

  FIELD_WRAPPER(int, 0, nomadz_msgs::msg::PersonalData::id_reciever_pass, idRecieverPass), // Number of the receiving robot

  FIELD_WRAPPER(bool,
                false,
                nomadz_msgs::msg::PersonalData::ready_to_kick,
                readyToKick), // Tell to the module if it has to perform calculation or not

  // Determines if the player wants a ball and (if yes) what score it assigns

  FIELD_WRAPPER(bool, false, nomadz_msgs::msg::PersonalData::wants_ball, wantsBall),
  FIELD_WRAPPER(float, INFINITY, nomadz_msgs::msg::PersonalData::ball_score, ballScore),

  // Kick estimate

  FIELD_WRAPPER(float, 0, nomadz_msgs::msg::PersonalData::last_kick_time, lastKickTime),
  FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::PersonalData::kick_estimate, kickEstimate),
});
