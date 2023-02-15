/**
 * @file Ally.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once
#include "Detection.h"

class Ally {
public:
  unsigned robotNumber;
  Pose2D state;
  Matrix2x2<> covar;
  bool hasBeenDetected = false;
  std::vector<unsigned> lastSeen = std::vector<unsigned>(TeamMateData::numOfPlayers);

  Ally() {}

  void update(RobotPose selfLoc, unsigned robotNum) {
    state = selfLoc;
    covar[0][0] = selfLoc.covariance[0][0];
    covar[0][1] = selfLoc.covariance[0][1];
    covar[1][0] = selfLoc.covariance[1][0];
    covar[1][1] = selfLoc.covariance[1][1];
    robotNumber = robotNum;
  }

  float distance(Detection& detect) { return (state.translation - detect.getPosition()).abs(); }

  void merge(Detection& detect) {
    Matrix2x2<> covPlusSensorCov = covar;
    covPlusSensorCov += detect.getCovar();
    Matrix2x2<> k = covar * covPlusSensorCov.invert();
    Vector2<> innovation = detect.getPosition() - state.translation;
    Vector2<> correction = k * innovation;
    state.translate(correction);
    covar -= k * covar;
  }
};
