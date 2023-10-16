/**
 *@file Detection.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once
#include "Ally.h"
#include "CombinedWorldModelProvider.h"

class Assignment {
public:
  enum TYPE { NONE, ALLY, OPPONENT };
  float dist;
  int robot;
  int nDetection;
};
class Detection {
public:
  unsigned timeStamp; /* Timestamp of the Detection. */
  Assignment::TYPE assignmentType = Assignment::NONE;
  int assignedTo = 0;
  float distance;
  std::vector<Assignment> assignments;
  Detection() {} // Constructor

  Detection(const Pose2D& observer,
            const Matrix2x2<>& observerCovar,
            const Vector2<>& robotPosition,
            const Matrix2x2<>& measurementCovariance,
            unsigned timeStamp)
      : position(observer * robotPosition), variance(rotate(measurementCovariance, observer.rotation) + observerCovar),
        timeStamp(timeStamp) {}

  Vector2<> getPosition() { return position; }

  Matrix2x2<> getCovar() { return variance; }

  Vector2f getPositionf() { return Vector2f(position.x, position.y); }

  Matrix2x2f getCovarf() {
    Matrix2x2f result;
    result[0][0] = variance[0][0];
    result[0][1] = variance[0][1];
    result[1][0] = variance[1][0];
    result[1][1] = variance[1][1];
    return result;
  }

private:
  Vector2<> position;
  Matrix2x2<> variance;

  static Matrix2x2<> rotate(const Matrix2x2<>& matrix, const float angle) {
    const float cosine = std::cos(angle);
    const float sine = std::sin(angle);
    const Matrix2x2<> rotationMatrix(cosine, -sine, sine, cosine);
    return (rotationMatrix * matrix) * rotationMatrix.transpose();
  }
};
