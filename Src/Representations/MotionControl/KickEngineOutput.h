/**
 * @file KickEngineOutput.h
 *
 * This file declares a struct that represents the output of modules generating motion.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:judy@tzi.de">Judith Müller</A>
 */

#pragma once

#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/KickRequest.h"
#include "Core/Math/Pose2D.h"
#include "Core/Math/Pose3D.h"

/**
 * @struct KickEngineOutput
 * A struct that represents the output of the kick engine.
 */

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/kick_engine_output.hpp"
#endif
STREAMABLE_DECLARE(KickEngineOutput)

STREAMABLE_ROS(
  KickEngineOutput,
  {
    ,
    FIELD_WRAPPER_DEFAULT(JointRequest,
                          nomadz_msgs::msg::KickEngineOutput::joint_request,
                          jointRequest), /**< Joint request for the kick. */
    FIELD_WRAPPER_DEFAULT(Pose2D,
                          nomadz_msgs::msg::KickEngineOutput::odometry_offset,
                          odometryOffset), /**< The body motion performed in this step. */
    FIELD_WRAPPER(bool,
                  true,
                  nomadz_msgs::msg::KickEngineOutput::is_leaving_possible,
                  isLeavingPossible), /**< Is leaving the motion module possible now? */
    FIELD_WRAPPER(bool, true, nomadz_msgs::msg::KickEngineOutput::is_stable, isStable), /**< Is motion currently stable? */
    FIELD_WRAPPER_DEFAULT(KickRequest,
                          nomadz_msgs::msg::KickEngineOutput::executed_kick_request,
                          executedKickRequest), /**< The kick request that is currently in execution. */
  });
