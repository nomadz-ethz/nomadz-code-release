/**
 * @file KickRequest.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once
#include "Core/Math/Vector3.h"
#include "Core/Enum.h"
#include "Core/Streams/AutoStreamable.h"
#include "Core/Streams/FieldWrapper.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/dyn_point.hpp"
#endif
STREAMABLE_DECLARE(DynPoint)

STREAMABLE_ROS(DynPoint, {
public:
  DynPoint(int limb,
           int phaseNumber,
           int duration,
           const Vector3<>& translation,
           const Vector3<>& angle,
           const Vector3<>& odometryOffset);
  DynPoint(int limb, int phaseNumber, const Vector3<>& translationl, const int duration = -1);

  bool operator==(const DynPoint& other) const, FIELD_WRAPPER_DEFAULT(int, nomadz_msgs::msg::DynPoint::limb, limb),
    FIELD_WRAPPER_DEFAULT(int, nomadz_msgs::msg::DynPoint::phase_number, phaseNumber),
    FIELD_WRAPPER_DEFAULT(int, nomadz_msgs::msg::DynPoint::duration, duration),
    FIELD_WRAPPER_DEFAULT(Vector3<>, nomadz_msgs::msg::DynPoint::translation, translation),
    FIELD_WRAPPER_DEFAULT(Vector3<>, nomadz_msgs::msg::DynPoint::angle, angle),
    FIELD_WRAPPER_DEFAULT(Vector3<>, nomadz_msgs::msg::DynPoint::odometry_offset, odometryOffset),
});

inline bool DynPoint::operator==(const DynPoint& other) const {
  return limb == other.limb && phaseNumber == other.phaseNumber && duration == other.duration &&
         translation == other.translation && angle == other.angle && odometryOffset == other.odometryOffset;
};

inline DynPoint::DynPoint(int limb,
                          int phaseNumber,
                          int duration,
                          const Vector3<>& translation,
                          const Vector3<>& angle,
                          const Vector3<>& odometryOffset)
    : DynPoint() {
  this->limb = limb;
  this->phaseNumber = phaseNumber;
  this->duration = duration;
  this->translation = translation;
  this->angle = angle;
  this->odometryOffset = odometryOffset;
}

inline DynPoint::DynPoint(int limb, int phaseNumber, const Vector3<>& translation, const int duration) : DynPoint() {
  this->limb = limb;
  this->phaseNumber = phaseNumber;
  this->duration = duration;
  this->translation = translation;
}

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/kick_request.hpp"
#endif
STREAMABLE_DECLARE(KickRequest)

STREAMABLE_ROS(KickRequest, {
public:
  ENUM(KickMotionID,
       kickForward,
       slowKick,
       fastKick,
       fastKickSlower,
       bhumanKick,
       kickPass,
       omniKickDefault,
       omniKickLong,
       newKick,
       none);
  static KickMotionID getKickMotionFromName(const char*name),
    FIELD_WRAPPER(KickMotionID, none, nomadz_msgs::msg::KickRequest::kick_motion_type, kickMotionType),
    FIELD_WRAPPER(float, 0, nomadz_msgs::msg::KickRequest::kick_angle, kickAngle),
    FIELD_WRAPPER(float, 0.5, nomadz_msgs::msg::KickRequest::kick_power, kickPower),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::KickRequest::mirror, mirror),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::KickRequest::auto_proceed, autoProceed),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::KickRequest::arms_back_fix, armsBackFix),
    FIELD_WRAPPER(bool, false, nomadz_msgs::msg::KickRequest::boost, boost),
    FIELD_WRAPPER_DEFAULT(std::vector<DynPoint>, nomadz_msgs::msg::KickRequest::dyn_points, dynPoints),
});

class Continuation : public Streamable {
public:
  KickRequest::KickMotionID kickType = KickRequest::none;
  bool mirror = false;

  static const char* getName(KickRequest::KickMotionID k) { return KickRequest::getName(k); }

private:
  virtual void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(kickType);
    STREAM(mirror);
    STREAM_REGISTER_FINISH;
  }
};

using stdVectorContinuation = std::vector<Continuation>;
