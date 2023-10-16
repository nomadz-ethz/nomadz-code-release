/**
 * @file RobotDimensions.h
 *
 * Description of the Dimensions of the NAO Robot
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Cord Niehaus
 */

#pragma once

#include "Core/Math/Vector3.h"
#include "Core/Streams/AutoStreamable.h"

#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/robot_dimensions.hpp"
#endif
STREAMABLE_DECLARE(RobotDimensions)

STREAMABLE_ROS(RobotDimensions,
{
public:
  /**
   * forward offset between head tilt joint and current camera
   * @param lowerCamera true, if lower camera is in use, false otherwise.
   */
  float getXHeadTiltToCamera(bool lowerCamera) const {return lowerCamera ? xHeadTiltToCamera : xHeadTiltToUpperCamera;
}

/**
 * height offset between head tilt joint and current camera
 * @param lowerCamera true, if lower camera is in use, false otherwise.
 */
float getZHeadTiltToCamera(bool lowerCamera) const {
  return lowerCamera ? zHeadTiltToCamera : zHeadTiltToUpperCamera;
}

/**
 * tilt of current camera against head tilt
 * @param lowerCamera true, if lower camera is in use, false otherwise.
 */
float getHeadTiltToCameraTilt(bool lowerCamera) const {
  return lowerCamera ? headTiltToCameraTilt : headTiltToUpperCameraTilt;
}
, FIELD_WRAPPER(float, 0.f, nomadz_msgs::msg::RobotDimensions::y_hip_offset, yHipOffset), //!< The y offset of the left hip.

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::length_between_legs,
                lengthBetweenLegs), //!< length between leg joints LL1 and LR1

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::upper_leg_length,
                upperLegLength), //!< length between leg joints LL2 and LL3 in z-direction

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::lower_leg_length,
                lowerLegLength), //!< length between leg joints LL3 and LL4 in z-direction

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::height_leg5_joint,
                heightLeg5Joint), //!< height of leg joints LL4 and LR4 of the ground

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::z_leg_joint1_to_head_pan,
                zLegJoint1ToHeadPan), //!< height offset between LL1 and head pan joint

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::x_head_tilt_to_camera,
                xHeadTiltToCamera), //!< forward offset between head tilt joint and lower camera

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::z_head_tilt_to_camera,
                zHeadTiltToCamera), //!< height offset between head tilt joint and lower camera

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::head_tilt_to_camera_tilt,
                headTiltToCameraTilt), //!< tilt of lower camera against head tilt

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::x_head_tilt_to_upper_camera,
                xHeadTiltToUpperCamera), //!< forward offset between head tilt joint and upper camera

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::z_head_tilt_to_upper_camera,
                zHeadTiltToUpperCamera), //!< height offset between head tilt joint and upper camera

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::head_tilt_to_upper_camera_tilt,
                headTiltToUpperCameraTilt), //!< tilt of upper camera against head tilt

  FIELD_WRAPPER_DEFAULT(Vector3<>,
                        nomadz_msgs::msg::RobotDimensions::arm_offset,
                        armOffset), //!< The offset of the first left arm joint relative to the middle between the hip joints

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::y_elbow_shoulder,
                yElbowShoulder), //!< The offset between the elbow joint and the shoulder joint in y

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::upper_arm_length,
                upperArmLength), //!< The length between the shoulder and the elbow in y-direction

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::lower_arm_length,
                lowerArmLength), //!< height off lower arm starting at arm2/arm3

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::x_offset_elbow_to_wrist,
                xOffsetElbowToWrist), //!< The length from Elbow to WristJoint.

  // for NDD kinematic

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::foot_front,
                footFront), //!< The offset of the foot joints to the foot tip.

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::foot_back,
                footBack), //!< The offset of the foot joints to the back of the foot.

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::foot_outer,
                footOuter), //!< The offset of the foot joints to the outer end of the foot.

  FIELD_WRAPPER(float,
                0.f,
                nomadz_msgs::msg::RobotDimensions::foot_inner,
                footInner), //!< The offset of the foot joints to the inner end of the foot.
});
