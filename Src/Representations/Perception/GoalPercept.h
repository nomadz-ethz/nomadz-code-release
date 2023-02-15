/**
 * @file GoalPercept.h
 *
 * Representation of a seen goal
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 */

#pragma once

#include "Core/Math/Vector2.h"
#include "Core/Enum.h"
#include "Core/Streams/AutoStreamable.h"

/**
 * @class GoalPost
 * Description of a perceived goal post
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/goal_post.hpp"
#endif
STREAMABLE_DECLARE(GoalPost)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/goal_percept.hpp"
#endif
STREAMABLE_DECLARE(GoalPercept)

STREAMABLE_ROS(GoalPost, {
  public : ENUM(Position, IS_UNKNOWN, IS_LEFT, IS_RIGHT),
  FIELD_WRAPPER(Position, IS_UNKNOWN, nomadz_msgs::msg::GoalPost::position, position), /**< position of this post */
  FIELD_WRAPPER_DEFAULT(Vector2<int>,
                        nomadz_msgs::msg::GoalPost::position_in_image,
                        positionInImage), /**< The position of the goal post in the current image */
  FIELD_WRAPPER_DEFAULT(Vector2<float>,
                        nomadz_msgs::msg::GoalPost::position_on_field,
                        positionOnField), /**< The position of the goal post relative to the robot*/
});

/**
 * @class GoalPercept
 * Set of perceived goal posts
 */
STREAMABLE_ROS(GoalPercept, {
  public :
    /** Draws the perceived goal posts*/
    void draw() const,
  FIELD_WRAPPER_DEFAULT(std::vector<GoalPost>, nomadz_msgs::msg::GoalPercept::goal_posts, goalPosts),
  FIELD_WRAPPER(unsigned,
                0,
                nomadz_msgs::msg::GoalPercept::time_when_goal_post_last_seen,
                timeWhenGoalPostLastSeen), /**< Time when a goal post was seen. */
  FIELD_WRAPPER(unsigned,
                0,
                nomadz_msgs::msg::GoalPercept::time_when_complete_goal_last_seen,
                timeWhenCompleteGoalLastSeen), /**< Time when complete goal was seen. */
});
