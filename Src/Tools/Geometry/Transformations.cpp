/**
 * @file Transformations.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 */

#include "Transformations.h"

#include "Core/Math/Pose2D.h"
#include "Core/Math/Pose3D.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/FieldBoundary.h"

namespace {
  const double MAX_DIST_ON_FIELD = sqrt(10400.f * 10400.f + 7400.f * 7400.f);

  bool calculatePointByAngles(const Vector2<>& angles,
                              const CameraMatrix& cameraMatrix,
                              const CameraInfo& cameraInfo,
                              Vector2<int>& point) {
    Vector3<> vectorToPointWorld, vectorToPoint;
    vectorToPointWorld.x = cos(angles.x);
    vectorToPointWorld.y = sin(angles.x);
    vectorToPointWorld.z = tan(angles.y);

    RotationMatrix rotationMatrix = cameraMatrix.rotation;
    vectorToPoint = rotationMatrix.invert() * vectorToPointWorld;

    float factor = cameraInfo.focalLength;

    float scale = factor / vectorToPoint.x;

    point.x = (int)(0.5f + cameraInfo.opticalCenter.x - vectorToPoint.y * scale);
    point.y = (int)(0.5f + cameraInfo.opticalCenter.y - vectorToPoint.z * scale);
    return vectorToPoint.x > 0;
  }
} // namespace

Vector2<> Geometry::relative2FieldCoord(const Pose2D& rp, float x, float y) {
  return rp * Vector2<>(x, y);
}

Vector2<> Geometry::relative2FieldCoord(const Pose2D& rp, const Vector2<>& relPosOnField) {
  return rp * relPosOnField;
}

Vector2<> Geometry::fieldCoord2Relative(const Pose2D& robotPose, const Vector2<>& fieldCoord) {
  // return robotPose.invert() * fieldCoord;

  const float invRotation = -robotPose.rotation;
  const float s = sin(invRotation);
  const float c = cos(invRotation);
  const float x = robotPose.translation.x;
  const float y = robotPose.translation.y;
  return Vector2<>(c * (fieldCoord.x - x) - s * (fieldCoord.y - y), s * (fieldCoord.x - x) + c * (fieldCoord.y - y));
}

float Geometry::getDistanceBySize(const CameraInfo& cameraInfo, float sizeInReality, float sizeInPixels) {
  float xFactor = cameraInfo.focalLength;
  return sizeInReality * xFactor / (sizeInPixels + 0.000001f);
}

float Geometry::getDistanceBySize(
  const CameraInfo& cameraInfo, float sizeInReality, float sizeInPixels, float centerX, float centerY) {
  float mx = centerX;
  float my = centerY;
  float cx = cameraInfo.opticalCenter.x;
  float cy = cameraInfo.opticalCenter.y;
  float focalLenPow2 = cameraInfo.focalLenPow2;
  float sqrImgRadius = (mx - cx) * (mx - cx) + (my - cy) * (my - cy);
  float imgDistance = sqrt(focalLenPow2 + sqrImgRadius);
  return imgDistance * sizeInReality / (sizeInPixels + 0.000001f);
}

bool Geometry::calculatePointOnField(
  const int x, const int y, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2<>& pointOnField) {
  Vector3<> vectorToCenter(1,
                           float(cameraInfo.opticalCenter.x - x) * cameraInfo.focalLengthInv,
                           float(cameraInfo.opticalCenter.y - y) * cameraInfo.focalLengthInv);
  Vector3<> vectorToCenterWorld = cameraMatrix.rotation * vectorToCenter;

  // Is the point above the horizon ? - return
  if (vectorToCenterWorld.z > -5 * cameraInfo.focalLengthInv) {
    return false;
  }

  const float f = cameraMatrix.translation.z / vectorToCenterWorld.z;

  pointOnField.x = cameraMatrix.translation.x - f * vectorToCenterWorld.x;
  pointOnField.y = cameraMatrix.translation.y - f * vectorToCenterWorld.y;

  return std::abs(pointOnField.x) < MAX_DIST_ON_FIELD && std::abs(pointOnField.y) < MAX_DIST_ON_FIELD;
}

bool Geometry::calculatePointOnField(
  const float x, const float y, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2<>& pointOnField) {
  Vector3<> vectorToCenter(1,
                           float(cameraInfo.opticalCenter.x - x) * cameraInfo.focalLengthInv,
                           float(cameraInfo.opticalCenter.y - y) * cameraInfo.focalLengthInv);
  Vector3<> vectorToCenterWorld = cameraMatrix.rotation * vectorToCenter;

  // Is the point above the horizon ? - return
  if (vectorToCenterWorld.z > -5 * cameraInfo.focalLengthInv) {
    return false;
  }

  const float f = cameraMatrix.translation.z / vectorToCenterWorld.z;

  pointOnField.x = cameraMatrix.translation.x - f * vectorToCenterWorld.x;
  pointOnField.y = cameraMatrix.translation.y - f * vectorToCenterWorld.y;

  return std::abs(pointOnField.x) < MAX_DIST_ON_FIELD && std::abs(pointOnField.y) < MAX_DIST_ON_FIELD;
}

bool Geometry::calculatePointOnField(const Vector2<>& point,
                                     const CameraMatrix& cameraMatrix,
                                     const CameraInfo& cameraInfo,
                                     Vector2<>& pointOnField) {
  Vector3<> vectorToCenter(1,
                           float(cameraInfo.opticalCenter.x - point.x) * cameraInfo.focalLengthInv,
                           float(cameraInfo.opticalCenter.y - point.y) * cameraInfo.focalLengthInv);
  Vector3<> vectorToCenterWorld = cameraMatrix.rotation * vectorToCenter;

  // Is the point above the horizon ? - return
  if (vectorToCenterWorld.z > -5 * cameraInfo.focalLengthInv) {
    return false;
  }

  const float f = cameraMatrix.translation.z / vectorToCenterWorld.z;

  pointOnField.x = cameraMatrix.translation.x - f * vectorToCenterWorld.x;
  pointOnField.y = cameraMatrix.translation.y - f * vectorToCenterWorld.y;

  return std::abs(pointOnField.x) < MAX_DIST_ON_FIELD && std::abs(pointOnField.y) < MAX_DIST_ON_FIELD;
}

bool Geometry::calculatePointOnField(
  const int x, const int y, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2<int>& pointOnField) {
  Vector2<> pointOnFieldDouble;
  bool onField = calculatePointOnField(x, y, cameraMatrix, cameraInfo, pointOnFieldDouble);
  pointOnField.x = (int)pointOnFieldDouble.x;
  pointOnField.y = (int)pointOnFieldDouble.y;
  return onField;
}

bool Geometry::calculatePointOnField(const Vector2<>& image,
                                     const float& fieldZ,
                                     const CameraMatrix& cameraMatrix,
                                     const CameraInfo& cameraInfo,
                                     Vector3<>& field) {
  Vector3<> unscaledCamera(
    cameraInfo.focalLength, float(cameraInfo.opticalCenter.x - image.x), float(cameraInfo.opticalCenter.y - image.y));
  const Vector3<> unscaledField(cameraMatrix.rotation * unscaledCamera);

  if (fieldZ < cameraMatrix.translation.z) {
    if (unscaledField.z > 0) {
      return false;
    }
  } else {
    if (unscaledField.z < 0) {
      return false;
    }
  }

  const float scale((cameraMatrix.translation.z - (float)fieldZ) / unscaledField.z);
  field.x = cameraMatrix.translation.x - scale * unscaledField.x;
  field.y = cameraMatrix.translation.y - scale * unscaledField.y;
  field.z = (float)fieldZ;
  return true;
}

bool Geometry::calculatePointInImage(const Vector2<>& point,
                                     const CameraMatrix& cameraMatrix,
                                     const CameraInfo& cameraInfo,
                                     Vector2<int>& pointInImage) {
  Vector2<> offset(point.x - cameraMatrix.translation.x, point.y - cameraMatrix.translation.y);
  return calculatePointByAngles(Vector2<>(atan2(offset.y, offset.x), -atan2(cameraMatrix.translation.z, offset.abs())),
                                cameraMatrix,
                                cameraInfo,
                                pointInImage);
}

bool Geometry::calculatePointsInImage(const std::vector<Vector2<>>& points,
                                      const CameraMatrix& cameraMatrix,
                                      const CameraInfo& cameraInfo,
                                      std::vector<Vector2<int>>& pointsInImage) {
  bool result = true;
  for (const Vector2<>& point : points) {
    Vector2<int> pointInImage;
    result &= calculatePointInImage(point, cameraMatrix, cameraInfo, pointInImage);
    pointsInImage.push_back(pointInImage);
  }
  return result;
}

bool Geometry::calculatePointInImage(const Vector3<>& point,
                                     const CameraMatrix& cameraMatrix,
                                     const CameraInfo& cameraInfo,
                                     Vector2<>& pointInImage) {
  Vector3<> pointInCam = cameraMatrix.invert() * point;
  if (pointInCam.x == 0) {
    return false;
  }
  pointInCam *= cameraInfo.focalLength / pointInCam.x;
  pointInImage = cameraInfo.opticalCenter - Vector2<>(pointInCam.y, pointInCam.z);
  return pointInCam.x > 0;
}

bool Geometry::calculatePointInImage(const Vector3<>& pointInWorld,
                                     const CameraMatrix& cameraMatrix,
                                     const CameraInfo& cameraInfo,
                                     Vector2<int>& pointInImage) {
  Vector3<> pointInCam = cameraMatrix.invert() * pointInWorld;
  if (pointInCam.x == 0) {
    return false;
  }
  pointInCam *= cameraInfo.focalLength / pointInCam.x;
  pointInImage = Vector2<int>(cameraInfo.opticalCenter - Vector2<>(pointInCam.y, pointInCam.z));
  return pointInCam.x > 0;
}

// Compute position in image of line (p1 -> p2) as (img1 -> img2)
// Returns false if line is entirely behind camera image plane
bool Geometry::calculateLineInImage(const Vector2<>& p1,
                                    const Vector2<>& p2,
                                    const Pose3D& cameraMatrixInv,
                                    const CameraInfo& cameraInfo,
                                    Vector2<int>& img1,
                                    Vector2<int>& img2) {
  const float f = cameraInfo.focalLength;

  // points are transformed into camera coordinates
  Vector3<> p1Camera = cameraMatrixInv * Vector3<>(p1.x, p1.y, 0.f);
  Vector3<> p2Camera = cameraMatrixInv * Vector3<>(p2.x, p2.y, 0.f);

  // handle the case that points can lie behind the camera plane
  bool p1Behind = p1Camera.x < f;
  bool p2Behind = p2Camera.x < f;

  if (p1Behind && p2Behind) {
    return false;

  } else if (!p1Behind && !p2Behind) {
    // both rays can be simply intersected with the image plane
    p1Camera /= (p1Camera.x / f);
    p2Camera /= (p2Camera.x / f);

  } else {
    // if one point lies behind the camera and the other in front, there must be an intersection of the connective line
    // with the image plane
    const Vector3<> direction = p1Camera - p2Camera;
    const float scale = (f - p1Camera.x) / direction.x;
    const Vector3<> intersection = p1Camera + direction * scale;

    if (p1Behind) {
      p1Camera = intersection;
      p2Camera /= (p2Camera.x / f);

    } else {
      p2Camera = intersection;
      p1Camera /= (p1Camera.x / f);
    }
  }

  img1.x = int(cameraInfo.opticalCenter.x - p1Camera.y);
  img1.y = int(cameraInfo.opticalCenter.y - p1Camera.z);
  img2.x = int(cameraInfo.opticalCenter.x - p2Camera.y);
  img2.y = int(cameraInfo.opticalCenter.y - p2Camera.z);

  return true;
}

bool Geometry::calculateLineInImage(
  const Line& line, const Pose3D& cameraMatrixInv, const CameraInfo& cameraInfo, Vector2<int>& img1, Vector2<int>& img2) {
  return calculateLineInImage(line.base, line.base + line.direction, cameraMatrixInv, cameraInfo, img1, img2);
}

Geometry::Line Geometry::calculateHorizon(const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo) {
  Line horizon;
  float r31 = cameraMatrix.rotation.c0.z;
  float r32 = cameraMatrix.rotation.c1.z;
  float r33 = cameraMatrix.rotation.c2.z;

  if (r33 == 0) {
    r33 = 0.00001f;
  }

  float x1 = 0, x2 = float(cameraInfo.width - 1), v1 = cameraInfo.focalLength, v2 = cameraInfo.opticalCenter.x,
        v3 = cameraInfo.opticalCenter.y, y1 = (v3 * r33 + r31 * v1 + r32 * v2) / r33,
        y2 = (v3 * r33 + r31 * v1 - r32 * v2) / r33;

  // Mirror ends of horizon if Camera rotated to the left
  if ((cameraMatrix.rotation * Vector3<>(0, 0, 1)).z < 0) {
    float t = x1;
    x1 = x2;
    x2 = t;
    t = y1;
    y1 = y2;
    y2 = t;
  }

  horizon.base.x = (x1 + x2) / 2.0f;
  horizon.base.y = (y1 + y2) / 2.0f;
  horizon.direction.x = x2 - x1;
  horizon.direction.y = y2 - y1;
  horizon.normalizeDirection();
  return horizon;
}

float Geometry::angleSizeToPixelSize(float angleSize, const CameraInfo& cameraInfo) {
  return cameraInfo.focalLength * tan(angleSize);
}

float Geometry::pixelSizeToAngleSize(float pixelSize, const CameraInfo& cameraInfo) {
  return atan(pixelSize * cameraInfo.focalLengthInv);
}

float Geometry::getSizeByDistance(const CameraInfo& cameraInfo, float sizeInReality, float distance) {
  float xFactor = cameraInfo.focalLength;
  return sizeInReality / distance * xFactor;
}

void Geometry::calculateAnglesForPoint(const Vector2<>& point,
                                       const CameraMatrix& cameraMatrix,
                                       const CameraInfo& cameraInfo,
                                       Vector2<>& angles) {
  float factor = cameraInfo.focalLength;

  Vector3<> vectorToPoint(factor, cameraInfo.opticalCenter.x - point.x, cameraInfo.opticalCenter.y - point.y);

  Vector3<> vectorToPointWorld = cameraMatrix.rotation * vectorToPoint;

  angles.x = atan2(vectorToPointWorld.y, vectorToPointWorld.x);

  angles.y = atan2(vectorToPointWorld.z, sqrt(sqr(vectorToPointWorld.x) + sqr(vectorToPointWorld.y)));
}

// Returns closed polygon on field inside which camera can see, in robot-centric field coordinates
// "Inside" is on left side of (point[i] -> point[i+1]) vector of each segment
std::vector<Vector2<>> Geometry::computeFovQuadrangle(const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo) {
  Vector3<> vectorToCenter(1, 0, 0);

  std::vector<Vector2<>> p(4);

  const float farAway = 12000.f;

  RotationMatrix r = cameraMatrix.rotation;
  r.rotateY(cameraInfo.openingAngleHeight / 2);
  r.rotateZ(cameraInfo.openingAngleWidth / 2);
  Vector3<> vectorToCenterWorld = r * vectorToCenter;

  float a1 = cameraMatrix.translation.x, a2 = cameraMatrix.translation.y, a3 = cameraMatrix.translation.z,
        b1 = vectorToCenterWorld.x, b2 = vectorToCenterWorld.y, b3 = vectorToCenterWorld.z, f = a3 / b3;
  Vector2<> pof = Vector2<>(a1 - f * b1, a2 - f * b2);

  if (f > 0.f) {
    p[0] = Vector2<>(0.f, 0.f);
  } else {
    p[0] = pof;
  }

  r = cameraMatrix.rotation;
  r.rotateY(cameraInfo.openingAngleHeight / 2);
  r.rotateZ(-(cameraInfo.openingAngleWidth / 2));
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x;
  b2 = vectorToCenterWorld.y;
  b3 = vectorToCenterWorld.z;
  f = a3 / b3;
  pof = Vector2<>(a1 - f * b1, a2 - f * b2);

  if (f > 0.f) {
    p[1] = Vector2<>(0.f, 0.f);
  } else {
    p[1] = pof;
  }

  r = cameraMatrix.rotation;
  r.rotateY(-(cameraInfo.openingAngleHeight / 2));
  r.rotateZ(-(cameraInfo.openingAngleWidth / 2));
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x;
  b2 = vectorToCenterWorld.y;
  b3 = vectorToCenterWorld.z;
  f = a3 / b3;
  pof = Vector2<>(a1 - f * b1, a2 - f * b2);

  if (f > 0.f) {
    p[2] = Vector2<>(farAway, 0).rotate((-cameraInfo.openingAngleWidth / 2) + cameraMatrix.rotation.getZAngle());
  } else {
    p[2] = pof;
  }

  r = cameraMatrix.rotation;
  r.rotateY(-(cameraInfo.openingAngleHeight / 2));
  r.rotateZ(cameraInfo.openingAngleWidth / 2);
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x;
  b2 = vectorToCenterWorld.y;
  b3 = vectorToCenterWorld.z;
  f = a3 / b3;
  pof = Vector2<>(a1 - f * b1, a2 - f * b2);

  if (f > 0.f) {
    p[3] = Vector2<>(farAway, 0).rotate((cameraInfo.openingAngleWidth / 2) + cameraMatrix.rotation.getZAngle());
  } else {
    p[3] = pof;
  }

  return p;
}

float Geometry::angleTo(const Pose2D& from, const Vector2<>& to) {
  Pose2D relPos = Pose2D(to) - from;
  return atan2(relPos.translation.y, relPos.translation.x);
}

float Geometry::distanceTo(const Pose2D& from, const Vector2<>& to) {
  return (Pose2D(to) - from).translation.abs();
}

Vector2<> Geometry::vectorTo(const Pose2D& from, const Vector2<>& to) {
  return (Pose2D(to) - from).translation;
}

bool Geometry::calculatePixelRatioOnField(
  const float x, const float y, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, float& ratio) {
  Vector2<> pw1, pw2, ph1, ph2;
  if (Geometry::calculatePointOnField(x - 0.5f, y, cameraMatrix, cameraInfo, pw1) &&
      Geometry::calculatePointOnField(x + 0.5f, y, cameraMatrix, cameraInfo, pw2) &&
      Geometry::calculatePointOnField(x, y - 0.5f, cameraMatrix, cameraInfo, ph1) &&
      Geometry::calculatePointOnField(x, y + 0.5f, cameraMatrix, cameraInfo, ph2)) {
    ratio = sqrt((pw1 - pw2).squareAbs() / (ph1 - ph2).squareAbs());
    return true;
  }
  return false;
}

bool Geometry::estimateInvPixelRatioOnField(const float y,
                                            const CameraMatrix& cameraMatrix,
                                            const CameraInfo& cameraInfo,
                                            float& ratio) {
  const float f = cameraInfo.focalLength;
  const float th0 = atan2(cameraMatrix.rotation.c0.x, -cameraMatrix.rotation.c0.z);
  const float th1 = atan2(cameraInfo.opticalCenter.y + 0.5f - y, f) + th0; // 0.5px up
  const float th2 = atan2(cameraInfo.opticalCenter.y - 0.5f - y, f) + th0; // 0.5px down
  if (th2 > 0 && th1 < M_PI / 2.f) {
    ratio = f * (tan(th1) - tan(th2)) * cos(th1);
    return true;
  }
  return false;
}

bool Geometry::calculatePixelInsideBoundary(const int x,
                                            const int y,
                                            const FieldBoundary& fieldBoundary,
                                            const CameraInfo& cameraInfo) {
  return Geometry::calculatePixelInsideBoundary(static_cast<float>(x), static_cast<float>(y), fieldBoundary, cameraInfo);
};
bool Geometry::calculatePixelInsideBoundary(const float x,
                                            const float y,
                                            const FieldBoundary& fieldBoundary,
                                            const CameraInfo& cameraInfo) {
  float y_boundary = Geometry::findBoundaryY(x, fieldBoundary, cameraInfo);
  return y_boundary < y * 1.0f;
};

float Geometry::findBoundaryY(const float p, const FieldBoundary& fieldBoundary, const CameraInfo& cameraInfo) {
  float x = (0.5f + p) * 1.0f / cameraInfo.width;
  if (fieldBoundary.model[1] == -1) {
    return std::max(0.0f, std::min(1.0f, fieldBoundary.model[0] + (fieldBoundary.model[3] - fieldBoundary.model[0]) * x)) *
           cameraInfo.height;
  }
  if (x < fieldBoundary.model[1]) {
    return std::max(0.0f,
                    std::min(1.0f,
                             fieldBoundary.model[0] +
                               (fieldBoundary.model[2] - fieldBoundary.model[0]) * x / fieldBoundary.model[1])) *
           cameraInfo.height;
  } else {
    return std::max(0.0f,
                    std::min(1.0f,
                             fieldBoundary.model[2] + (fieldBoundary.model[3] - fieldBoundary.model[2]) *
                                                        (x - fieldBoundary.model[1]) / (1 - fieldBoundary.model[1]))) *
           cameraInfo.height;
  }
}