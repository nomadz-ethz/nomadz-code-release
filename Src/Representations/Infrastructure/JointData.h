/**
 * @file JointData.h
 *
 * This file declares a classes to represent the joint angles.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Core/Math/Common.h"
#include "Core/Streams/AutoStreamable.h"
#include "Core/Enum.h"

/**
 * @class JointData
 * A class to represent the joint angles sent to the robot.
 */
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/joint_data.hpp"
#endif
STREAMABLE_DECLARE(JointData)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/hardness_data.hpp"
#endif
STREAMABLE_DECLARE(HardnessData)

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/joint_request.hpp"
#endif
STREAMABLE_DECLARE(JointRequest)

STREAMABLE_ROS(JointData, {
public:
  ENUM(Joint,
       HeadYaw,
       HeadPitch,
       FirstArmJoint,
       FirstLeftArmJoint = FirstArmJoint,
       LShoulderPitch = FirstLeftArmJoint,
       LShoulderRoll,
       LElbowYaw,
       LElbowRoll,
       LWristYaw,
       LHand, ///< not an Angle, instead %
       FirstRightArmJoint,
       RShoulderPitch = FirstRightArmJoint,
       RShoulderRoll,
       RElbowYaw,
       RElbowRoll,
       RWristYaw,
       RHand, ///< not an angle, instead %
       FirstLegJoint,
       FirstLeftLegJoint = FirstLegJoint,
       LHipYawPitch = FirstLeftLegJoint,
       LHipRoll,
       LHipPitch,
       LKneePitch,
       LAnklePitch,
       LAnkleRoll,
       FirstRightLegJoint,
       RHipYawPitch = FirstRightLegJoint, ///< not a joint in the real nao
       RHipRoll,
       RHipPitch,
       RKneePitch,
       RAnklePitch,
       RAnkleRoll);

  // If you change those values be sure to change them in MofCompiler.cpp too. (Line ~280)
  enum { off = 1000 };    /**< Special angle for switching off a joint. */
  enum { ignore = 2000 }; /**< Special angle for not overwriting the previous setting. */

  /**
   * The method returns the angle of the mirror (left/right) of the given joint.
   * @param joint The joint the mirror of which is returned.
   * @return The angle of the mirrored joint.
   */
  float mirror(Joint joint) const {
    switch (joint) {
    // don't mirror an invalid joint value (!)
    case HeadYaw:
      return angles[HeadYaw] == off || angles[HeadYaw] == ignore ? angles[HeadYaw] : -(angles[HeadYaw]);
    case LShoulderPitch:
      return angles[RShoulderPitch];
    case LShoulderRoll:
      return angles[RShoulderRoll] == off || angles[RShoulderRoll] == ignore ? angles[RShoulderRoll]
                                                                             : -(angles[RShoulderRoll]);
    case LElbowYaw:
      return angles[RElbowYaw] == off || angles[RElbowYaw] == ignore ? angles[RElbowYaw] : -(angles[RElbowYaw]);
    case LElbowRoll:
      return angles[RElbowRoll] == off || angles[RElbowRoll] == ignore ? angles[RElbowRoll] : -(angles[RElbowRoll]);
    case LWristYaw:
      return angles[RWristYaw] == off || angles[RWristYaw] == ignore ? angles[RWristYaw] : -(angles[RWristYaw]);
    case LHand:
      return angles[RHand];
    case RShoulderPitch:
      return angles[LShoulderPitch];
    case RShoulderRoll:
      return angles[LShoulderRoll] == off || angles[LShoulderRoll] == ignore ? angles[LShoulderRoll]
                                                                             : -(angles[LShoulderRoll]);
    case RElbowYaw:
      return angles[LElbowYaw] == off || angles[LElbowYaw] == ignore ? angles[LElbowYaw] : -(angles[LElbowYaw]);
    case RElbowRoll:
      return angles[LElbowRoll] == off || angles[LElbowRoll] == ignore ? angles[LElbowRoll] : -(angles[LElbowRoll]);
    case RWristYaw:
      return angles[LWristYaw] == off || angles[LWristYaw] == ignore ? angles[LWristYaw] : -(angles[LWristYaw]);
    case RHand:
      return angles[LHand];
    case LHipYawPitch:
      return angles[RHipYawPitch];
    case LHipRoll:
      return angles[RHipRoll] == off || angles[RHipRoll] == ignore ? angles[RHipRoll] : -(angles[RHipRoll]);
    case LHipPitch:
      return angles[RHipPitch];
    case LKneePitch:
      return angles[RKneePitch];
    case LAnklePitch:
      return angles[RAnklePitch];
    case LAnkleRoll:
      return angles[RAnkleRoll] == off || angles[RAnkleRoll] == ignore ? angles[RAnkleRoll] : -(angles[RAnkleRoll]);
    case RHipYawPitch:
      return angles[LHipYawPitch];
    case RHipRoll:
      return angles[LHipRoll] == off || angles[LHipRoll] == ignore ? angles[LHipRoll] : -(angles[LHipRoll]);
    case RHipPitch:
      return angles[LHipPitch];
    case RKneePitch:
      return angles[LKneePitch];
    case RAnklePitch:
      return angles[LAnklePitch];
    case RAnkleRoll:
      return angles[LAnkleRoll] == off || angles[LAnkleRoll] == ignore ? angles[LAnkleRoll] : -(angles[LAnkleRoll]);
    default:
      return angles[joint];
    }
  }
  /**
   * The method initializes the joint angles as a mirror of a set of other joint angles.
   * @param other The set of joint angles that are mirrored.
   */
  void mirror(const JointData& other) {
    for (int i = 0; i < numOfJoints; ++i)
      angles[i] = other.mirror((Joint)i);
    timeStamp = other.timeStamp;
  }
  , FIELD_WRAPPER_DEFAULT(float[numOfJoints], nomadz_msgs::msg::JointData::angles, angles), /**< The angles of all joints. */
    FIELD_WRAPPER(
      unsigned, 0, nomadz_msgs::msg::JointData::time_stamp, timeStamp), /**< The time when these angles were received. */

    // Initialization
    for (int i = 0; i < numOfJoints; ++i) angles[i] = off;
});

/**
 * @class HardnessData
 * This class represents the joint hardness in a jointRequest.
 * It loads the default hardness values from hardnessSettings.cfg.
 */
STREAMABLE_ROS(HardnessData, {
public:
  enum { useDefault = -1 };

  /**
   * The method returns the hardness of the mirror (left/right) of the given joint.
   * @param joint The joint the mirror of which is returned.
   * @return The output hardness of the mirrored joint.
   */
  int mirror(const JointData::Joint joint) const {
    switch (joint) {
    case JointData::HeadYaw:
      return hardness[JointData::HeadYaw];
    case JointData::LShoulderPitch:
      return hardness[JointData::RShoulderPitch];
    case JointData::LShoulderRoll:
      return hardness[JointData::RShoulderRoll];
    case JointData::LElbowYaw:
      return hardness[JointData::RElbowYaw];
    case JointData::LElbowRoll:
      return hardness[JointData::RElbowRoll];
    case JointData::LWristYaw:
      return hardness[JointData::RWristYaw];
    case JointData::LHand:
      return hardness[JointData::RHand];
    case JointData::RShoulderPitch:
      return hardness[JointData::LShoulderPitch];
    case JointData::RShoulderRoll:
      return hardness[JointData::LShoulderRoll];
    case JointData::RElbowYaw:
      return hardness[JointData::LElbowYaw];
    case JointData::RElbowRoll:
      return hardness[JointData::LElbowRoll];
    case JointData::RWristYaw:
      return hardness[JointData::LWristYaw];
    case JointData::RHand:
      return hardness[JointData::LHand];
    case JointData::LHipYawPitch:
      return hardness[JointData::RHipYawPitch];
    case JointData::LHipRoll:
      return hardness[JointData::RHipRoll];
    case JointData::LHipPitch:
      return hardness[JointData::RHipPitch];
    case JointData::LKneePitch:
      return hardness[JointData::RKneePitch];
    case JointData::LAnklePitch:
      return hardness[JointData::RAnklePitch];
    case JointData::LAnkleRoll:
      return hardness[JointData::RAnkleRoll];
    case JointData::RHipYawPitch:
      return hardness[JointData::LHipYawPitch];
    case JointData::RHipRoll:
      return hardness[JointData::LHipRoll];
    case JointData::RHipPitch:
      return hardness[JointData::LHipPitch];
    case JointData::RKneePitch:
      return hardness[JointData::LKneePitch];
    case JointData::RAnklePitch:
      return hardness[JointData::LAnklePitch];
    case JointData::RAnkleRoll:
      return hardness[JointData::LAnkleRoll];
    default:
      return hardness[joint];
    }
  }

  /**
   * initializes this instance with the mirrored values of other
   * @param other the HardnessData to be mirrored
   */
  void mirror(const HardnessData& other) {
    for (int i = 0; i < JointData::numOfJoints; ++i)
      hardness[i] = other.mirror((JointData::Joint)i);
  }

  /**
   * This function resets the hardness for all joints to the default value.
   */
  inline void resetToDefault() {
    for (int i = 0; i < JointData::numOfJoints; ++i)
      hardness[i] = useDefault;
  }
  ,
    FIELD_WRAPPER_DEFAULT(int[JointData::numOfJoints],
                          nomadz_msgs::msg::HardnessData::hardness,
                          hardness), /**< the custom hardness for each joint */

    // Initialization
    resetToDefault();
});

STREAMABLE_ALIAS(HardnessSettings, HardnessData, {});

/**
 * @class JointRequest
 */
STREAMABLE_ROS(JointRequest, 
{
public:
  /**
   * Initializes this instance with the mirrored data from a other JointRequest
   * @param other the JointRequest to be mirrored
   */
  void mirror(const JointRequest& other)
  {
    jointAngles.mirror(other.jointAngles);
    jointHardness.mirror(other.jointHardness);
}

/**
 * Returns the mirrored angle of joint
 * @param joint the joint to be mirrored
 */
float mirror(const JointData::Joint joint) {
  return jointAngles.mirror(joint);
}

bool isValid() const {
  for (int i = 0; i < JointData::numOfJoints; ++i)
    if (std::isnan(jointAngles.angles[i]) || jointHardness.hardness[i] < 0 || jointHardness.hardness[i] > 100)
      return false;
  return true;
}
,
  FIELD_WRAPPER_DEFAULT(JointData,
                        nomadz_msgs::msg::JointRequest::joint_angles,
                        jointAngles), /**< the angles for all joints */
  FIELD_WRAPPER_DEFAULT(HardnessData,
                        nomadz_msgs::msg::JointRequest::joint_hardness,
                        jointHardness), /**< the hardness for all joints */
});

STREAMABLE_ALIAS(FilteredJointData, JointData, {});
STREAMABLE_ALIAS(NonArmeMotionEngineOutput, JointRequest, {});
