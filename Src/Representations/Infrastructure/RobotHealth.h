/**
 * @file RobotHealth.h
 *
 * The file declares two classes to transport information about the current robot health
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Core/Debugging/DebugDrawings.h"

/**
 * @class MotionRobotHealth
 * All information collected within motion process.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/robot_health.hpp"
#endif
STREAMABLE_DECLARE(RobotHealth)

STREAMABLE_ROS(RobotHealth, {
public:
  void draw() const {
    DECLARE_PLOT("representation:RobotHealth:batteryLevel");
    PLOT("representation:RobotHealth:batteryLevel", batteryLevel);
    DECLARE_PLOT("representation:RobotHealth:maxJointTemperature");
    PLOT("representation:RobotHealth:maxJointTemperature", maxJointTemperature);
    DECLARE_PLOT("representation:RobotHealth:totalCurrent");
    PLOT("representation:RobotHealth:totalCurrent", totalCurrent);
  }
  ,
    FIELD_WRAPPER(float,
                  0,
                  nomadz_msgs::msg::RobotHealth::motion_frame_rate,
                  motionFrameRate), /*< Frames per second within process "Motion" */
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::RobotHealth::avg_motion_time, avgMotionTime), /**< average execution time */
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::RobotHealth::max_motion_time, maxMotionTime), /**< Maximum execution time */
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::RobotHealth::min_motion_time, minMotionTime), /**< Minimum execution time */
    FIELD_WRAPPER(float,
                  0,
                  nomadz_msgs::msg::RobotHealth::cognition_frame_rate,
                  cognitionFrameRate), /*< Frames per second within process "Cognition" */
    FIELD_WRAPPER(unsigned char,
                  0,
                  nomadz_msgs::msg::RobotHealth::battery_level,
                  batteryLevel), /*< Current batteryLevel of robot battery in percent */
    FIELD_WRAPPER(float,
                  0,
                  nomadz_msgs::msg::RobotHealth::total_current,
                  totalCurrent), /*< Sum of all motor currents ( as a measure for the robot's current load) */
    FIELD_WRAPPER(unsigned char,
                  0,
                  nomadz_msgs::msg::RobotHealth::max_joint_temperature,
                  maxJointTemperature), /*< Highest temperature of a robot actuator */
    FIELD_WRAPPER(
      unsigned char, 0, nomadz_msgs::msg::RobotHealth::cpu_temperature, cpuTemperature), /**< The temperature of the cpu */
    FIELD_WRAPPER(unsigned char,
                  0,
                  nomadz_msgs::msg::RobotHealth::board_temperature,
                  boardTemperature), /**< The temperature of the mainboard or northbridge, dunno */
    FIELD_WRAPPER_DEFAULT(unsigned char[3], nomadz_msgs::msg::RobotHealth::load, load), /*< load averages */
    FIELD_WRAPPER_DEFAULT(
      unsigned char, nomadz_msgs::msg::RobotHealth::memory_usage, memoryUsage), /*< Percentage of used memory */
    FIELD_WRAPPER_DEFAULT(std::string, nomadz_msgs::msg::RobotHealth::robot_name, robotName), /*< For fancier drawing :-) */
    FIELD_WRAPPER_DEFAULT(unsigned int,
                          nomadz_msgs::msg::RobotHealth::ball_percepts,
                          ballPercepts), /**< A ball percept counter used to determine ball percepts per hour */
    FIELD_WRAPPER_DEFAULT(unsigned int,
                          nomadz_msgs::msg::RobotHealth::line_analyses,
                          lineAnalyses), /**< A line percept counter used to determine line percepts per hour */
    FIELD_WRAPPER_DEFAULT(unsigned int,
                          nomadz_msgs::msg::RobotHealth::goal_percepts,
                          goalPercepts), /**< A goal percept counter used to determine goal percepts per hour */
    FIELD_WRAPPER_DEFAULT(
      bool, nomadz_msgs::msg::RobotHealth::wlan, wlan), /**< Status of the wlan hardware. true: wlan
                                   hardware is ok. false: wlan hardware is (probably physically) broken. */

    // Initialization
    load[0] = load[1] = load[2] = 0;
});

STREAMABLE_ALIAS(MotionRobotHealth, RobotHealth, {});
