/**
 * @file WalkGeneratorData.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Infrastructure/JointData.h"
#include "Core/Function.h"
#include "Core/Math/Pose2D.h"
#include "Core/Math/Pose3D.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/walk_generator.hpp"
#endif
STREAMABLE_DECLARE(WalkGeneratorData)

STREAMABLE_WITH_BASE(WalkGeneratorData, WalkGeneratorDataBaseWrapper, {
public:
  ENUM(WalkMode,
       speedMode,    /**< Speed in mm/s and radians/s. */
       stepSizeMode, /**< Step size in mm and radians (speedScale is ignored). */
       targetMode,
       patternMode) /**< Walk to given relative position in mm and radians. */

  ENUM(WalkState, standing, starting, walking, stopping)
  ENUM(WeightShiftStatus, weightDidShift, weightDidNotShift, emergencyStep)
  /**
   * Initializes the generator. Must be called whenever the control is returned to this module after
   * another one was responsible for creating the motions. Must also be called once after creation.
   */
  FUNCTION(void()) reset;

  /**
   * Calculates a new set of joint angles to let the robot walk or stand. Must be called every 10 ms.
   * @param speed The speed or step size to walk with. If everything is zero, the robot stands.
   * @param target The target to walk to if in target mode.
   * @param walkMode How are speed and target interpreted?
   * @param getKickFootOffset If set, provides an offset to add to the pose of the swing foot to
   *                          create a kick motion. It must be suited for the foot that actually is
   *                          the swing foot.
   */
  FUNCTION(void(const Pose2D& speed, const Pose2D& target, WalkMode walkMode))
  calcJoints,
    FIELD_WRAPPER_DEFAULT(
      JointRequest, nomadz_msgs::msg::WalkGeneratorData::joint_request, jointRequest), /**< The calculated joint angles. */
    FIELD_WRAPPER_DEFAULT(Pose2D,
                          nomadz_msgs::msg::WalkGeneratorData::odometry_offset,
                          odometryOffset), /**< The relative motion in this frame is returned here (in mm and radians). */
    FIELD_WRAPPER_DEFAULT(
      Pose2D,
      nomadz_msgs::msg::WalkGeneratorData::upcoming_odometry_offset,
      upcomingOdometryOffset), /**< The minimum remaining odometry offset until the robot can come to a full stop. */
    FIELD_WRAPPER_LEGACY(Pose2D, Pose2D(45_deg, 100.f, 100.f), nomadz_msgs::msg::WalkGeneratorData::max_speed, maxSpeed),
    /**< The maximum speed possible. */
    FIELD_WRAPPER_DEFAULT(Pose2D,
                          nomadz_msgs::msg::WalkGeneratorData::speed,
                          speed), /**< The average speed during the current step in mm/s and radians/s. */
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::WalkGeneratorData::return_offset, returnOffset),
    FIELD_WRAPPER(float,
                  0.f,
                  nomadz_msgs::msg::WalkGeneratorData::step_duration,
                  stepDuration), /**< The expected duration of the current step (in s). */
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::WalkGeneratorData::t, t), /**< Current time in the walk cycle (in s). */
    FIELD_WRAPPER(
      bool, false, nomadz_msgs::msg::WalkGeneratorData::is_left_phase, isLeftPhase), /**< Is the left foot swinging? */

    // Additional Property from the Combination of WalkGeneratorData and WalkGeneratorData
    FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::WalkGeneratorData::turn, turn),
    FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::WalkGeneratorData::forward, forward), // current walk forward value

    FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::WalkGeneratorData::left, left),
    FIELD_WRAPPER_DEFAULT(Pose2D, nomadz_msgs::msg::WalkGeneratorData::current_speed, currentSpeed),
    FIELD_WRAPPER(int,
                  0,
                  nomadz_msgs::msg::WalkGeneratorData::used_predicted_switch,
                  usedPredictedSwitch), // counter for how many predicted foot support switches were made
    FIELD_WRAPPER_DEFAULT(WeightShiftStatus, nomadz_msgs::msg::WalkGeneratorData::weigth_shift_status, weightShiftStatus),
    FIELD_WRAPPER_DEFAULT(WalkState, nomadz_msgs::msg::WalkGeneratorData::walk_state, walkState),

    FIELD_WRAPPER_DEFAULT(Vector3<>, nomadz_msgs::msg::WalkGeneratorData::swing_control_point, swingControlPoint),
    FIELD_WRAPPER_DEFAULT(Vector3<>, nomadz_msgs::msg::WalkGeneratorData::swing_control_point0, swingControlPoint0),
    FIELD_WRAPPER_DEFAULT(Vector3<>, nomadz_msgs::msg::WalkGeneratorData::left_foot_offset0, leftFootOffset0),
    FIELD_WRAPPER_DEFAULT(Vector3<>, nomadz_msgs::msg::WalkGeneratorData::right_foot_offset0, rightFootOffset0),
    FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::WalkGeneratorData::turn_rl, turnRL0),
    FIELD_WRAPPER_DEFAULT(float, nomadz_msgs::msg::WalkGeneratorData::switch_phase, switchPhase),
    FIELD_WRAPPER_DEFAULT(Vector3<>, nomadz_msgs::msg::WalkGeneratorData::left_ref_point, leftRefPoint),
    FIELD_WRAPPER_DEFAULT(Vector3<>, nomadz_msgs::msg::WalkGeneratorData::right_ref_point, rightRefPoint),
});
