/**
 * @file CombinedWorldModel.h
 *
 * Declaration of a representation that represents a combined world model
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Katharina Gillmann
 */

#pragma once

#include <vector>
#include "Representations/Modeling/BallModel.h"
#include "Core/Math/Matrix2x2.h"
#include "Core/Math/Pose2D.h"

/**
 * @class GaussianDistribution
 * a gaussian distribution consisting of a mean and a covariance.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/gaussian_position_distribution.hpp"
#endif
STREAMABLE_DECLARE(GaussianPositionDistribution)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/combined_world_model.hpp"
#endif
STREAMABLE_DECLARE(CombinedWorldModel)

STREAMABLE_ROS(GaussianPositionDistribution, {
  public : GaussianPositionDistribution(const Vector2<>& robotPosition, const Matrix2x2<>& covariance),
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::GaussianPositionDistribution::robot_position,
                        robotPosition), /**< Position (mean) of the detected robot, mean is the center point in both cases
(ultrasonic and vision) */
  FIELD_WRAPPER_DEFAULT(Matrix2x2<>,
                        nomadz_msgs::msg::GaussianPositionDistribution::covariance,
                        covariance), /**< covariance of the mesasurement **/
});

/**
 * @class CombinedWorldModel
 * a combined world model.
 */
STREAMABLE_ROS(CombinedWorldModel, {
  public : void draw() const,

  // std::vector<bool> does not work :(

  FIELD_WRAPPER_DEFAULT(std::vector<unsigned char>,
                        nomadz_msgs::msg::CombinedWorldModel::can_play,
                        canPlay), /**< true if this player can play (1-indexed) (not penalized, still on network) */
  FIELD_WRAPPER_DEFAULT(std::vector<unsigned char>,
                        nomadz_msgs::msg::CombinedWorldModel::can_trust,
                        canTrust), /**< true if can trust information from this player (1-indexed) (not penalized, still
on network, ground contact) */
  FIELD_WRAPPER_DEFAULT(std::vector<Pose2D>,
                        nomadz_msgs::msg::CombinedWorldModel::own_poses,
                        ownPoses), /**< poses of own robots, 1-indexed by robot number */
  FIELD_WRAPPER_DEFAULT(std::vector<Pose2D>, nomadz_msgs::msg::CombinedWorldModel::positions_own_team, positionsOwnTeam),
  /**< poses of own robots; contains only those fully active */ // TODO Combine somehow with ownPoses instead

  FIELD_WRAPPER_DEFAULT(std::vector<GaussianPositionDistribution>,
                        nomadz_msgs::msg::CombinedWorldModel::positions_opponent_team,
                        positionsOpponentTeam), /**< positions of opponent robots */
  FIELD_WRAPPER_DEFAULT(std::vector<unsigned int>,
                        nomadz_msgs::msg::CombinedWorldModel::time_since_ball_last_seen_player,
                        timeSinceBallLastSeenPlayer), /**< time since ball last seen by a player */
  FIELD_WRAPPER_DEFAULT(
    BallState,
    nomadz_msgs::msg::CombinedWorldModel::ball_state,
    ballState), /**< position and velocity of the ball in field coordinates. (Do not trust comments of BallState here) */
  FIELD_WRAPPER(
    bool, false, nomadz_msgs::msg::CombinedWorldModel::ball_is_valid, ballIsValid), /**< ball state is valid, if true */
  FIELD_WRAPPER(unsigned int,
                INT_MAX,
                nomadz_msgs::msg::CombinedWorldModel::time_since_ball_last_seen,
                timeSinceBallLastSeen), /**< time since ball last seen by anyone in the team, in ms */
  FIELD_WRAPPER_DEFAULT(
    std::vector<unsigned int>,
    nomadz_msgs::msg::CombinedWorldModel::time_since_ball_others_last_seen_player,
    timeSinceBallOthersLastSeenPlayer), /**< time since ball last seen by the ball seen by teammates only */
  FIELD_WRAPPER_DEFAULT(BallState,
                        nomadz_msgs::msg::CombinedWorldModel::ball_state_others,
                        ballStateOthers), /**< position and velocity of the ball as seen by teammates only */
  FIELD_WRAPPER(unsigned int,
                INT_MAX,
                nomadz_msgs::msg::CombinedWorldModel::time_since_ball_last_seen_others,
                timeSinceBallLastSeenOthers), /**< time since ball last seen by teammates only, in ms */
  FIELD_WRAPPER(bool,
                false,
                nomadz_msgs::msg::CombinedWorldModel::ball_is_valid_others,
                ballIsValidOthers), /**< if calculated ball state is valid as seen by teammates only */
  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::CombinedWorldModel::ball_state_others_max_side_confidence,
                ballStateOthersMaxSideConfidence), /**< Maximum SideConfidence of the involved robots */
  FIELD_WRAPPER_DEFAULT(Vector2<>, nomadz_msgs::msg::CombinedWorldModel::expected_end_position, expectedEndPosition),
  /**< expected end position of the ball. */
});
