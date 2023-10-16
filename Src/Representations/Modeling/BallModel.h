/**
 * @file BallModel.h
 *
 * Declaration of class BallModel
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include <bitset>
#include <climits>

#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Core/Math/Matrix2x2.h"
#include "Core/Streams/AutoStreamable.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/ball_state.hpp"
#endif
STREAMABLE_DECLARE(BallState);

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/ball_model.hpp"
#endif
STREAMABLE_DECLARE(BallModel);

class Pose2D;

/**
 * @class BallState
 *
 * Base class for ball position and velocity.
 */
STREAMABLE_ROS(BallState,
{
public:
  /**
  * Computes the position were a rolling ball is expected to stop rolling.
  * @return The position relative to the robot (in mm)
  */
  Vector2<> getEndPosition(float ballFriction) const
  {
    return position + velocity * (-1 / ballFriction);
}

/**
 * Computes the distance the ball will still be rolling
 */
float distanceToEndPosition(float ballFriction) const {
  return (position - getEndPosition(ballFriction)).abs();
}

/**
 * Computes the time (in seconds) the ball needs to pass distance.
 * Return std::numeric_limits<float>::max(), if ball won't make the distance with its current velocity.
 */
float timeForDistance(float distance, float ballFriction) const {
  if (distanceToEndPosition(ballFriction) < distance)
    return std::numeric_limits<float>::max();
  else
    return std::log(distance * ballFriction / velocity.abs() + 1) / ballFriction;
}
,
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::BallState::position,
                        position), /**< The position of the ball relative to the robot (in mm)*/
  FIELD_WRAPPER_DEFAULT(Vector2<>,
                        nomadz_msgs::msg::BallState::velocity,
                        velocity), /**< The velocity of the ball relative to the robot (in mm/s)*/
  FIELD_WRAPPER_DEFAULT(Matrix2x2<>,
                        nomadz_msgs::msg::BallState::position_covariance,
                        positionCovariance), /**< The covariance matrix describing the position uncertainty */
});

/**
 * @class BallModel
 *
 * Contains all current knowledge about the ball.
 */
STREAMABLE_ROS(BallModel, {
public:
  static const size_t bufferLength = 20; /**< How many entries to keep */
  std::bitset<bufferLength>
    seenBuffer; /**< Whether ball was seen (1 or 0) in the last few seconds (1 entry per upper & lower) */

  /** Draws the estimate on the field */
  void draw() const;

  /** Draws the estimate in the camera image */
  void drawImage(const CameraMatrix&, const CameraInfo&) const;

  /** Draws the end position of the estimate on the field */
  void drawEndPosition(float ballFriction) const;

  /** Draw the ball model in scene */
  void draw3D(const Pose2D&robotPose) const,
    FIELD_WRAPPER_DEFAULT(
      Vector2<>, nomadz_msgs::msg::BallModel::last_perception, lastPerception), /**< The last seen position of the ball */
    FIELD_WRAPPER_DEFAULT(
      BallState,
      nomadz_msgs::msg::BallModel::estimate,
      estimate), /**< The state of the ball estimated from own observations; it is propagated even if the ball is not seen */
    FIELD_WRAPPER(unsigned,
                  INT_MAX,
                  nomadz_msgs::msg::BallModel::time_when_last_seen,
                  timeWhenLastSeen), /**< Time stamp that it was seen in multiple frames */
    FIELD_WRAPPER(unsigned,
                  INT_MAX,
                  nomadz_msgs::msg::BallModel::time_when_last_detected,
                  timeWhenLastDetected), /**< Time stamp when last detection takes place */
    FIELD_WRAPPER(
      unsigned,
      INT_MAX,
      nomadz_msgs::msg::BallModel::time_when_disappeared,
      timeWhenDisappeared), /**< The time when the ball was not seen in the image altough it should have been there */
    FIELD_WRAPPER(unsigned,
                  INT_MAX,
                  nomadz_msgs::msg::BallModel::time_since_last_seen,
                  timeSinceLastSeen), /**< The time in milliseconds since time the ball was last seen */
    FIELD_WRAPPER_DEFAULT(float,
                          nomadz_msgs::msg::BallModel::seen_fraction,
                          seenFraction), /**< How often ball was seen in last few seconds, between 0 and 1 */
    FIELD_WRAPPER_DEFAULT(bool, nomadz_msgs::msg::BallModel::valid, valid), /**< Whether the ball is considered valid */
    FIELD_WRAPPER_DEFAULT(bool, nomadz_msgs::msg::BallModel::lost, lost),   /**< Whether the ball is considered lost */
});

/**
 * @class GroundTruthBallModel
 * The same as the BallModel, but - in general - provided by an external
 * source that has ground truth quality
 */
STREAMABLE_ALIAS(GroundTruthBallModel, BallModel, {
public:
  /** Draws something*/
  void draw() const;
});

STREAMABLE_ALIAS(BallModelAfterPreview, BallModel, {});

/**
 * @class BallModelCompressed
 * A compressed version of BallModel used in team communication
 */
STREAMABLE_DECLARE_LEGACY(BallModelCompressed)
STREAMABLE_LEGACY(BallModelCompressed, {
public:
  BallModelCompressed(const BallModel& ballModel);
  operator BallModel() const,

    (Vector2<short>)lastPerception,                       /**< The last seen position of the ball */
    (Vector2<short>)position, (unsigned)timeWhenLastSeen, /**< Time stamp, indicating what its name says */
    (float)seenFraction, /**< How often ball was seen in last few seconds, between 0 and 1 */
});
