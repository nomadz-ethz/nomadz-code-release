/**
 * @file JointCalibration.h
 *
 * Declaration of a class for representing the calibration values of joints.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Representations/Infrastructure/JointData.h"
#include "Core/Math/Common.h"

STREAMABLE_DECLARE_LEGACY(JointCalibration)
STREAMABLE_DECLARE_LEGACY(JointInfo)

class JointCalibration : public JointCalibrationBaseWrapper {
private:
  virtual void serialize(In* in, Out* out) {
    JointInfo& headYaw = joints[JointData::HeadYaw];
    JointInfo& headPitch = joints[JointData::HeadPitch];
    JointInfo& lShoulderPitch = joints[JointData::LShoulderPitch];
    JointInfo& lShoulderRoll = joints[JointData::LShoulderRoll];
    JointInfo& lElbowYaw = joints[JointData::LElbowYaw];
    JointInfo& lElbowRoll = joints[JointData::LElbowRoll];
    JointInfo& lWristYaw = joints[JointData::LWristYaw];
    JointInfo& lHand = joints[JointData::LHand];
    JointInfo& rShoulderPitch = joints[JointData::RShoulderPitch];
    JointInfo& rShoulderRoll = joints[JointData::RShoulderRoll];
    JointInfo& rElbowYaw = joints[JointData::RElbowYaw];
    JointInfo& rElbowRoll = joints[JointData::RElbowRoll];
    JointInfo& rWristYaw = joints[JointData::RWristYaw];
    JointInfo& rHand = joints[JointData::RHand];
    JointInfo& lHipYawPitch = joints[JointData::LHipYawPitch];
    JointInfo& lHipRoll = joints[JointData::LHipRoll];
    JointInfo& lHipPitch = joints[JointData::LHipPitch];
    JointInfo& lKneePitch = joints[JointData::LKneePitch];
    JointInfo& lAnklePitch = joints[JointData::LAnklePitch];
    JointInfo& lAnkleRoll = joints[JointData::LAnkleRoll];
    JointInfo& rHipYawPitch = joints[JointData::RHipYawPitch];
    JointInfo& rHipRoll = joints[JointData::RHipRoll];
    JointInfo& rHipPitch = joints[JointData::RHipPitch];
    JointInfo& rKneePitch = joints[JointData::RKneePitch];
    JointInfo& rAnklePitch = joints[JointData::RAnklePitch];
    JointInfo& rAnkleRoll = joints[JointData::RAnkleRoll];

    STREAM_REGISTER_BEGIN;
    STREAM(headYaw)
    STREAM(headPitch)
    STREAM(lShoulderPitch)
    STREAM(lShoulderRoll)
    STREAM(lElbowYaw)
    STREAM(lElbowRoll)
    STREAM(lWristYaw)
    STREAM(lHand)
    STREAM(rShoulderPitch)
    STREAM(rShoulderRoll)
    STREAM(rElbowYaw)
    STREAM(rElbowRoll)
    STREAM(rWristYaw)
    STREAM(rHand)
    STREAM(lHipYawPitch)
    STREAM(lHipRoll)
    STREAM(lHipPitch)
    STREAM(lKneePitch)
    STREAM(lAnklePitch)
    STREAM(lAnkleRoll)
    STREAM(rHipYawPitch)
    STREAM(rHipRoll)
    STREAM(rHipPitch)
    STREAM(rKneePitch)
    STREAM(rAnklePitch)
    STREAM(rAnkleRoll)
    STREAM_REGISTER_FINISH
  }

public:
  class JointInfo : public JointInfoBaseWrapper {
  private:
    virtual void serialize(In* in, Out* out) {
      float offset = toDegrees(this->offset);
      float maxAngle = toDegrees(this->maxAngle);
      float minAngle = toDegrees(this->minAngle);
      float jointPlay = toDegrees(this->jointPlay);

      STREAM_REGISTER_BEGIN;
      STREAM(offset);
      STREAM(minAngle);
      STREAM(maxAngle);
      STREAM(jointPlay);
      STREAM_REGISTER_FINISH

      if (in) {
        this->offset = fromDegrees(offset);
        this->minAngle = fromDegrees(minAngle);
        this->maxAngle = fromDegrees(maxAngle);
        this->jointPlay = fromDegrees(jointPlay);
      }
    }

  public:
    float offset;   /**< An offset added to the angle. */
    float maxAngle; /** the maximal angle in radians */
    float minAngle; /** the minmal angle in radians */
    float jointPlay;
    /**
     * Default constructor.
     */
    JointInfo() : offset(0), maxAngle(2.618f), minAngle(-2.618f), jointPlay(0) {}
  };

  JointInfo joints[JointData::numOfJoints]; /**< Information on the calibration of all joints. */
};
