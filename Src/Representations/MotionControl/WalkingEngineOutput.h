/**
 * @file WalkingEngineOutput.h
 *
 * This file declares a class that represents the output of modules generating motion.
 *
 *  This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/WalkRequest.h"
#include "Core/Math/Vector3.h"

/**
 * @class WalkingEnigeOutput
 * A class that represents the output of the walking engine.
 */

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/walking_engine_output.hpp"
#endif
STREAMABLE_DECLARE(WalkingEngineOutput)

STREAMABLE_ROS(
  WalkingEngineOutput,
  {
    ,
    FIELD_WRAPPER_DEFAULT(JointRequest,
                          nomadz_msgs::msg::WalkingEngineOutput::joint_request,
                          jointRequest), /**< Joint request for the walk. */
    FIELD_WRAPPER(bool,
                  true,
                  nomadz_msgs::msg::WalkingEngineOutput::standing,
                  standing), /**< Whether the robot is standing or walking */
    FIELD_WRAPPER_DEFAULT(Pose2D,
                          nomadz_msgs::msg::WalkingEngineOutput::speed,
                          speed), /**< The current walking speed in mm/s and rad/s. */
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::WalkingEngineOutput::max_speed, maxSpeed),
    FIELD_WRAPPER_DEFAULT(Pose2D,
                          nomadz_msgs::msg::WalkingEngineOutput::odometry_offset,
                          odometryOffset), /**< The body motion performed in this step. */
    FIELD_WRAPPER_DEFAULT(Pose2D,
                          nomadz_msgs::msg::WalkingEngineOutput::upcoming_odometry_offset,
                          upcomingOdometryOffset), /**< The remaining odometry offset for the currently executed step. */
    FIELD_WRAPPER_DEFAULT(Pose2D,
                          nomadz_msgs::msg::WalkingEngineOutput::offset_to_robot_pose_after_preview,
                          offsetToRobotPoseAfterPreview), /**< The offset after execution of preview phase. */
    FIELD_WRAPPER(bool,
                  false,
                  nomadz_msgs::msg::WalkingEngineOutput::upcoming_odometry_offset_valid,
                  upcomingOdometryOffsetValid), /**< Whether the \c upcomingOdometryOffset is precise enough to be used */
    FIELD_WRAPPER(bool,
                  true,
                  nomadz_msgs::msg::WalkingEngineOutput::is_leaving_possible,
                  isLeavingPossible), /**< Is leaving the motion module possible now? */
    FIELD_WRAPPER(float,
                  0,
                  nomadz_msgs::msg::WalkingEngineOutput::position_in_walk_cycle,
                  positionInWalkCycle), /**< The current position in the walk cycle in the range [0..1[. */
    FIELD_WRAPPER(float,
                  0,
                  nomadz_msgs::msg::WalkingEngineOutput::instability,
                  instability), /**< An evaluation of the current walk stability. */
    FIELD_WRAPPER_DEFAULT(WalkRequest,
                          nomadz_msgs::msg::WalkingEngineOutput::executed_walk,
                          executedWalk), /**< The walk currently executed. */
  });

/**
 * @class WalkingEngineOutput
 * A class that represents the output of the walking engine.
 */
STREAMABLE_ALIAS(WalkingEngineStandOutput, JointRequest, {});
