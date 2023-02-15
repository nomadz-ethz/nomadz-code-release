/**
 * @file RotationPoint.cpp
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once
#include "Tools/DortmundWalkingEngine/Graham.h"
#include <Eigen/Eigen>

#define LEFT_FOOT 0
#define RIGHT_FOOT 1

/**
 * @class RotationPoint
 * Calculates the point of rotation of the body
 * (a point of the convex hull of the support polygon).
 */
class RotationPoint {
public:
  /** Constructor */
  RotationPoint(void);
  /** Destructor */
  ~RotationPoint(void){};

  Eigen::Vector2f footHull[2][8]; /**< Given hull for left and right foot constisting of 8 points each */

  /**
   * Calculates the current axis of rotation
   * @param direction The tipping direction. Use the down direction (x and y).
   * @param doubleSupport True, if current walking phase is a double support
   * @param footNum Number of the foot on floor (if doubleSupport==false), 0 is left foot, 1 is right foot
   * @return Plane vector of rotation axis
   */
  Eigen::Vector2f getCurrentRotationPoint(Eigen::Vector2f direction, bool doubleSupport, int footNum);

private:
  GrahamScan dshull;    /**< Convex hull for double support */
  GrahamScan ssHull[2]; /**<  Convex hull for left single support and right single support */

  bool lineIntersection(Eigen::Vector2f p1, Eigen::Vector2f p2, Eigen::Vector2f p3, Eigen::Vector2f p4);
};
