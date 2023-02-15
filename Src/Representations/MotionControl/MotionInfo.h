/**
 * @file MotionInfo.h
 *
 * Definition of class MotionInfo.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Martin Lötzsch
 */

#pragma once

#include "MotionRequest.h"

/**
 * @class MotionInfo
 * The executed motion request and additional information about the motions which are executed by the Motion process.
 */
#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/motion_info.hpp"
#endif
STREAMABLE_DECLARE(MotionInfo)

STREAMABLE_ROS(MotionInfo, 
{
public:
  bool isStanding() const
  {
	return (motionRequest.motion == MotionRequest::specialAction
       && (motionRequest.specialActionRequest.specialAction == SpecialActionRequest::stand || motionRequest.specialActionRequest.specialAction == SpecialActionRequest::standHigh));
}
,
  FIELD_WRAPPER_DEFAULT(MotionRequest,
                        nomadz_msgs::msg::MotionInfo::motion_request,
                        motionRequest), /**< Associated motion request. */
  FIELD_WRAPPER(bool,
                false,
                nomadz_msgs::msg::MotionInfo::is_motion_stable,
                isMotionStable), /**< If true, the motion is stable, leading to a valid torso / camera matrix. */
  FIELD_WRAPPER_DEFAULT(Pose2D,
                        nomadz_msgs::msg::MotionInfo::upcoming_odometry_offset,
                        upcomingOdometryOffset), /**< The remaining odometry offset for the currently executed motion. */
  FIELD_WRAPPER_DEFAULT(Pose2D,
                        nomadz_msgs::msg::MotionInfo::offset_to_robot_pose_after_preview,
                        offsetToRobotPoseAfterPreview), /**< The same as above, but from Nao Devils. TODO: Figure out why
                                                           they renamed upcomingOdometryOffset to this */
});
