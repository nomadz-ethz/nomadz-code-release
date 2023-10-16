/**
 * @file GroundTruthWorldState.h
 *
 * The file declares a struct that contains the state of the world in the current simulation scene.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Core/Math/Pose2D.h"
#include "Core/Streams/AutoStreamable.h"
#include <vector>

/**
 * @struct GroundTruthWorldState
 * A struct that contains the state of the world in the current simulation scene.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/ground_truth_player.hpp"
#endif
STREAMABLE_DECLARE(GroundTruthPlayer)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/ground_truth_world_state.hpp"
#endif
STREAMABLE_DECLARE(GroundTruthWorldState)

STREAMABLE_ROS(GroundTruthWorldState, {
public:
  STREAMABLE_ROS(GroundTruthPlayer,
                 {
                   ,
                   FIELD_WRAPPER(int, 0, nomadz_msgs::msg::GroundTruthPlayer::number, number),
                   FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::GroundTruthPlayer::pose, pose),
                   FIELD_WRAPPER(bool, false, nomadz_msgs::msg::GroundTruthPlayer::upright, upright),
                 });

  using Player = GroundTruthPlayer;
  , FIELD_WRAPPER(int, 0, nomadz_msgs::msg::GroundTruthWorldState::time, time),
    FIELD_WRAPPER_DEFAULT(std::vector<Vector3<>>, nomadz_msgs::msg::GroundTruthWorldState::balls, balls),
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::GroundTruthWorldState::own_pose, ownPose),
    FIELD_WRAPPER_DEFAULT(std::vector<GroundTruthPlayer>, nomadz_msgs::msg::GroundTruthWorldState::teammates, teammates),
    FIELD_WRAPPER_DEFAULT(std::vector<GroundTruthPlayer>, nomadz_msgs::msg::GroundTruthWorldState::opponents, opponents),
});
