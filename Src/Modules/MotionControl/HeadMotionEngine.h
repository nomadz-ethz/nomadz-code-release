/**
 * @file HeadMotionEngine.h
 *
 * This file implements a module that creates head joint angles from desired head motion.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <a href="allli@informatik.uni-bremen.de">Alexander Härtl</a> and Colin Graf
 */

#pragma once

#include "Core/Math/Vector2.h"
#include "Core/Module/Module.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Geometry/Shapes.h"

MODULE(HeadMotionEngine)
REQUIRES(HeadAngleRequest)
REQUIRES(FilteredJointData)
REQUIRES(FrameInfo)
REQUIRES(JointCalibration)
REQUIRES(GroundContactState)
PROVIDES_WITH_MODIFY(HeadJointRequest)
END_MODULE

class HeadMotionEngine : public HeadMotionEngineBase {
private:
  float requestedPan;
  float requestedTilt;
  Vector2<> lastSpeed;
  Geometry::Circle deathPoints[4];

  /**
   * The update method to generate the head joint angles from desired head motion.
   */
  void update(HeadJointRequest& headJointRequest);

public:
  HeadMotionEngine();
};
