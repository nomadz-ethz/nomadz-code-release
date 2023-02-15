/**
 * @file BehaviorControlOutput.h
 *
 * Declaration of class BehaviorControlOutput
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Max Risler
 */

#pragma once

#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/Plan.h"

#include "Representations/Infrastructure/PersonalData.h" //Personal Data

/**
 * A class collecting the output from the behavior control module
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/behavior_control_output.hpp"
#endif
STREAMABLE_DECLARE(BehaviorControlOutput)

STREAMABLE_ROS(
  BehaviorControlOutput,
  {
    ,
    FIELD_WRAPPER_DEFAULT(ArmMotionRequest, nomadz_msgs::msg::BehaviorControlOutput::arm_motion_request, armMotionRequest),
    FIELD_WRAPPER_DEFAULT(MotionRequest, nomadz_msgs::msg::BehaviorControlOutput::motion_request, motionRequest),
    FIELD_WRAPPER_DEFAULT(HeadMotionRequest,
                          nomadz_msgs::msg::BehaviorControlOutput::head_motion_request,
                          headMotionRequest),
    FIELD_WRAPPER_DEFAULT(OwnTeamInfo, nomadz_msgs::msg::BehaviorControlOutput::own_team_info, ownTeamInfo),
    FIELD_WRAPPER_DEFAULT(RobotInfo, nomadz_msgs::msg::BehaviorControlOutput::robot_info, robotInfo),
    FIELD_WRAPPER_DEFAULT(GameInfo, nomadz_msgs::msg::BehaviorControlOutput::game_info, gameInfo),
    FIELD_WRAPPER_DEFAULT(BehaviorStatus, nomadz_msgs::msg::BehaviorControlOutput::behavior_status, behaviorStatus),
    FIELD_WRAPPER_DEFAULT(BehaviorLEDRequest,
                          nomadz_msgs::msg::BehaviorControlOutput::behavior_l_e_d_request,
                          behaviorLEDRequest),
    FIELD_WRAPPER_DEFAULT(ActivationGraph, nomadz_msgs::msg::BehaviorControlOutput::execution_graph, executionGraph),
    FIELD_WRAPPER_DEFAULT(PersonalData, nomadz_msgs::msg::BehaviorControlOutput::personal_data, personalData),
  });
