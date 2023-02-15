/**
 *@file TrackedRobot.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once
#include "Detection.h"

class TrackedRobot {
public:
  // tracking
  unsigned latestUpdate;
  unsigned firstSeen;
  std::vector<unsigned> lastSeen = std::vector<unsigned>(TeamMateData::numOfPlayers);

  TrackedRobot() {}

  TrackedRobot(Detection& detect) {
    state = HTransposed * detect.getPositionf();
    covar = HTransposed * detect.getCovarf() * H;
    covar[1][1] = 1;
    covar[3][3] = 1;
    firstSeen = detect.timeStamp;
    latestUpdate = firstSeen;
  }

  void predictMovement(unsigned timestep) {
    // increase covar of tracked opponents, such that new detections can be assigned despite of robot movement
    covar *= 1.1;
    // simple 2d movement model
    // Matrix4x4f F =
    //   Matrix4x4f(Vector4f(1, 0, 0, 0), Vector4f(timestep, 1, 0, 0), Vector4f(0, 0, 1, 0), Vector4f(0, 0, timestep, 1));
    // // Noise Matrix should Model the unknown inputs of the opponents
    // Matrix4x4f Q = Matrix4x4f(Vector4f(pow(timestep, 3) / 3, pow(timestep, 2) / 2, 0, 0),
    //                           Vector4f(pow(timestep, 2) / 2, timestep, 0, 0),
    //                           Vector4f(0, 0, pow(timestep, 3) / 3, pow(timestep, 2) / 2),
    //                           Vector4f(0, 0, pow(timestep, 2) / 2, timestep));
    // Q *= movementNoise;
    // state = F * state;
    // covar = F * covar * F.transpose() + Q;
  }

  float distance(Detection& detect) {
    Vector2f dist = (detect.getPositionf() - H * state);
    return dist.abs();
  }

  void merge(Detection& detect) {
    // calculate Kalman gain and update state + covar
    Matrix2x2f covPlusSensorCov = H * covar * HTransposed;
    covPlusSensorCov += detect.getCovarf();
    Matrix4x2f k = covar * HTransposed * covPlusSensorCov.invert();
    Vector2f innovation = detect.getPositionf() - H * state;
    state += k * innovation;
    covar -= k * H * covar;

    latestUpdate = std::max(latestUpdate, detect.timeStamp);
  }

  Vector2f getPosition() { return H * state; }
  Matrix<2, 2, float> getCovar() { return H * covar * HTransposed; }
  float getTotalVar() { return std::sqrt(covar.det()); }

  GaussianPositionDistribution getGPD() {
    GaussianPositionDistribution gpd;
    auto temp = H * state;
    gpd.robotPosition = Vector2<>(temp[0], temp[1]);
    auto temp2 = H * covar * HTransposed;
    gpd.covariance[0][0] = temp2[0][0];
    gpd.covariance[1][0] = temp2[1][0];
    gpd.covariance[0][1] = temp2[0][1];
    gpd.covariance[1][1] = temp2[1][1];
    return gpd;
  }

private:
  // movement model
  Vector4f state;
  Matrix4x4f covar;
  Matrix2x4f H = Matrix2x4f(Vector2f(1, 0), Vector2f(), Vector2f(0, 1), Vector2f());
  Matrix4x2f HTransposed = H.transpose();
  // float movementNoise = 0.000001; // amplitude of the movement noise
};
