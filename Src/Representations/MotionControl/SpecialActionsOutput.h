/**
 * @file SpecialActionsOutput.h
 *
 * This file declares a class that represents the output of the special actions module.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is  <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/SpecialActionRequest.h"
#include "Core/Math/Pose2D.h"

/**
 * @class SpecialActionsOutput
 * A class that represents the output of the special actions module.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/special_actions_output.hpp"
#endif
STREAMABLE_DECLARE(SpecialActionsOutput)

STREAMABLE_ROS(SpecialActionsOutput,
               {
                 ,
                 FIELD_WRAPPER_DEFAULT(JointRequest,
                                       nomadz_msgs::msg::SpecialActionsOutput::joint_request,
                                       jointRequest), /**< The associated joint request. */
                 FIELD_WRAPPER_DEFAULT(Pose2D,
                                       nomadz_msgs::msg::SpecialActionsOutput::odometry_offset,
                                       odometryOffset), /**< The body motion performed in this step. */
                 FIELD_WRAPPER(bool,
                               true,
                               nomadz_msgs::msg::SpecialActionsOutput::is_leaving_possible,
                               isLeavingPossible), /**< Is leaving the motion module possible now? */
                 FIELD_WRAPPER(bool,
                               false,
                               nomadz_msgs::msg::SpecialActionsOutput::is_motion_stable,
                               isMotionStable), /**< Is the position of

the camera directly related to the kinematic chain of joint angles? */
                 FIELD_WRAPPER(bool, false, nomadz_msgs::msg::SpecialActionsOutput::is_motion_done, isMotionDone),
                 FIELD_WRAPPER_DEFAULT(SpecialActionRequest,
                                       nomadz_msgs::msg::SpecialActionsOutput::executed_special_action,
                                       executedSpecialAction), /**< The special action currently executed. */
               });
