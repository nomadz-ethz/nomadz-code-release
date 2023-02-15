/**
 * @file RotationPoint.cpp
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#include "RotationPoint.h"

RotationPoint::RotationPoint() {
  footHull[LEFT_FOOT][0].x() = 0.037f;
  footHull[LEFT_FOOT][0].y() = 0.046f;
  footHull[LEFT_FOOT][1].x() = 0.062f;
  footHull[LEFT_FOOT][1].y() = 0.017f;
  footHull[LEFT_FOOT][2].x() = 0.067f;
  footHull[LEFT_FOOT][2].y() = -0.024f;
  footHull[LEFT_FOOT][3].x() = 0.038f;
  footHull[LEFT_FOOT][3].y() = -0.042f;
  footHull[LEFT_FOOT][4].x() = -0.08f;
  footHull[LEFT_FOOT][4].y() = -0.035f;
  footHull[LEFT_FOOT][5].x() = -0.096f;
  footHull[LEFT_FOOT][5].y() = -0.017f;
  footHull[LEFT_FOOT][6].x() = -0.095f;
  footHull[LEFT_FOOT][6].y() = 0.032f;
  footHull[LEFT_FOOT][7].x() = -0.076f;
  footHull[LEFT_FOOT][7].y() = 0.052f;

  for (int i = 0; i < 8; i++) {
    footHull[RIGHT_FOOT][i].y() = -footHull[LEFT_FOOT][i].y();
    footHull[RIGHT_FOOT][i].x() = footHull[LEFT_FOOT][i].x();
  }

  for (int i = 0; i < 8; i++) {
    ssHull[LEFT_FOOT].addPoint(footHull[LEFT_FOOT][i].x(), footHull[LEFT_FOOT][i].y());
    ssHull[RIGHT_FOOT].addPoint(footHull[RIGHT_FOOT][i].x(), footHull[RIGHT_FOOT][i].y());
  }

  ssHull[LEFT_FOOT].build_hull();
  ssHull[RIGHT_FOOT].build_hull();
}

Eigen::Vector2f RotationPoint::getCurrentRotationPoint(Eigen::Vector2f direction, bool doubleSupport, int footNum) {
  Eigen::Vector2f nullPoint;

  if (!doubleSupport) {
    for (unsigned int i = 0; i < ssHull[footNum].complete.size(); i++) {
      Eigen::Vector2f p1, p2;
      p1.x() = ssHull[footNum].complete[i].first;
      p1.y() = ssHull[footNum].complete[i].second;
      p2.x() = ssHull[footNum].complete[(i + 1) % ssHull[footNum].complete.size()].first;
      p2.y() = ssHull[footNum].complete[(i + 1) % ssHull[footNum].complete.size()].second;
      if (lineIntersection(p1, p2, nullPoint, direction)) {
        // Returns the middle of the line segmet of the support polygon
        return p1 + (p2 - p1) / 2;
      }
    }
  } else {
    // double support currently not supported
  }

  return nullPoint;
}

bool RotationPoint::lineIntersection(Eigen::Vector2f p1, Eigen::Vector2f p2, Eigen::Vector2f p3, Eigen::Vector2f p4) {
  float r, s, d;
  // Make sure the lines aren't parallel
  if ((p2.y() - p1.y()) / (p2.x() - p1.x()) != (p4.y() - p3.y()) / (p4.x() - p3.x())) {
    d = (((p2.x() - p1.x()) * (p4.y() - p3.y())) - (p2.y() - p1.y()) * (p4.x() - p3.x()));
    if (d != 0) {
      r = (((p1.y() - p3.y()) * (p4.x() - p3.x())) - (p1.x() - p3.x()) * (p4.y() - p3.y())) / d;
      s = (((p1.y() - p3.y()) * (p2.x() - p1.x())) - (p1.x() - p3.x()) * (p2.y() - p1.y())) / d;
      if (r >= 0 && r <= 1 && s >= 0) {
        // result.InsertSolution(p1.x() + r * (p2.x() - p1.x()), p1.y() + r * (p2.y() - p1.y()));
        return true;
      }
    }
  }
  return false;
}
