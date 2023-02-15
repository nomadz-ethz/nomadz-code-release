/**
 * @file WalkingInfo.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
 */

#pragma once

#include "Tools/DortmundWalkingEngine/StepData.h"
#include "Tools/DortmundWalkingEngine/WalkingInformations.h"
#include "Representations/Modeling/RobotPose.h"
#include "Core/Math/Vector.h"
#include "Core/Math/Vector2.h"
#ifndef WALKING_SIMULATOR
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Streams/Streamable.h"
#include "Core/Enum.h"
#include "Core/Debugging/Watch.h"
#else
#include "math/Pose2D.h"
#include "bhumanstub.h"
#endif

/**
 * @class WalkingInfo
 * Gives some information about the walk.
 */
STREAMABLE_DECLARE_LEGACY(WalkingInfo)
class WalkingInfo : public WalkingInfoBaseWrapper {
public:
  WalkingInfo() : isLeavingPossible(true), isRunning(false), bodyTiltApplied(false){};

  Pose2D odometryOffset;                /**< Distancte between last odometry position and current */
  Pose2D robotPosition;                 /**< Current position of body in world coordinate system of the walking engine */
  Pose2D offsetToRobotPoseAfterPreview; /**< Future position of robot after the preview phase */
  Vector2<float> expectedAcc;           /**< Expected acceleration of the body */
  bool isLeavingPossible;               /**< Is is possible to leave the walking engine without falling? */
  StepData lastUsedFootPositions;
  bool isInstantKickRunning;
  double accX_XOffset;
  Vector2<float> desiredBodyRot;
  bool isRunning;
  bool bodyTiltApplied;
  bool onFloor[2];

  Vector2<float> ballCSinWEWCS;

  void drawFoot(Point s, int t) const {
    Vector2<float> bodyPoints[4] = {
      Vector2<float>(45, 25), Vector2<float>(45, -25), Vector2<float>(-45, -25), Vector2<float>(-45, 25)};
    // Point s = f.footPos[ZMP::phaseToZMPFootMap[f.phase]];
    Pose2D p = s;
    p.translation *= 1000;
    p = walkingCStoSelfLocRCS(p);
    for (int i = 0; i < 4; i++)
      bodyPoints[i] = p * bodyPoints[i];
    POLYGON("module:SwingLegController:steps",
            4,
            bodyPoints,
            4,
            Drawings::ps_dash,
            ColorClasses::black,
            Drawings::solidBrush,
            ColorRGBA((t % 9) * 255 / 9, 255 - (t % 9) * 255 / 9, (t % 9) * 255 / 9));
  }

  Pose2D walkingCStoSelfLocRCS(Pose2D p) const {
    float rotDiff = -robotPosition.rotation;
    p.translation -= ballCSinWEWCS * 1000;
    p.translation.rotate(rotDiff);
    p.rotate(rotDiff);
    return p;
  }

  Pose2D selfLocRCStoWalkingCS(Pose2D p) const // untested
  {
    float rotDiff = robotPosition.rotation;
    p.translation.rotate(-rotDiff);
    p.rotate(-rotDiff);
    p.translation += ballCSinWEWCS * 1000;
    return p;
  }

  Vector2<float> ballCStoWalkingCS(Vector2<float> p) const {
    return p.rotate((float)robotPosition.rotation) / 1000 + ballCSinWEWCS;
  }

  Vector2<float> toWorldCoords(Vector2<float>& rcs) const { return robotPosition * rcs; }

  Vector2<float> vecToWorldCoords(Vector2<float>& rcs) const {
    Vector2<float> null = Vector2<float>(0, 0);
    return toWorldCoords(rcs) - toWorldCoords(null);
  }

  Point vecToWorldCoords(Point& rcs) const {
    Point null;
    return toWorldCoords(rcs) - toWorldCoords(null);
  }

  Pose2D toWorldCoords(Pose2D& rcs) const {
    Pose2D wcs(rcs.translation);
    wcs.rotate(robotPosition.rotation);
    wcs.translation = toWorldCoords(wcs.translation);
    return wcs;
  }

  Point toWorldCoords(const Point& rcs) const {
    Point wcs(rcs);
    wcs.rotate2D(robotPosition.rotation);
    return wcs + Point(robotPosition.translation.x, robotPosition.translation.y, 0, robotPosition.rotation);
  }

  Vector2<float> toRobotCoords(const Vector2<float>& wcs) const { return robotPosition.invert() * wcs; }

  Vector2<> vecToRobotCoords(Vector2<>& wcs) const { return toRobotCoords(wcs) - toRobotCoords(Vector2<float>(0, 0)); }

  Point vecToRobotCoords(const Point& wcs) const {
    Point null;
    return toRobotCoords(wcs) - toRobotCoords(null);
  }

  Pose2D toRobotCoords(Pose2D& wcs) const {
    Pose2D rcs = toRobotCoords(wcs.translation);
    rcs.rotate(-robotPosition.rotation);
    return rcs;
  }

  Point toRobotCoords(const Point& wcs) const {
    const Point rp(robotPosition.translation.x, robotPosition.translation.y, 0, robotPosition.rotation);

    Point rcs = wcs - rp;
    rcs.rotate2D(-rp.r);
    return rcs;
  }

protected:
  virtual void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(odometryOffset);
    STREAM(robotPosition);
    STREAM(offsetToRobotPoseAfterPreview);
    STREAM(expectedAcc);
    STREAM(isLeavingPossible);
    STREAM(isInstantKickRunning);
    STREAM(accX_XOffset);
    STREAM(desiredBodyRot)
    STREAM_REGISTER_FINISH;
  }
};
