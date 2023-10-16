/**
 * @file UKFSample.h
 *
 * Declaration of Unscented Kalman Filter for robot pose estimation
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a> and Colin Graf
 */

#pragma once

#include "Core/Math/Vector2.h"
#include "Core/Math/Matrix.h"
#include "Core/Math/Pose2D.h"
#include "Core/RingBufferWithSum.h"

class CameraMatrix;
class FieldModel;
class FieldDimensions;
class GoalPercept;
class LineAnalysis;

/**
 * @class UKFSample
 *
 * Hypothesis of a robot's pose, modeled as an Unscented Kalman Filter
 */
class UKFSample { // NOLINT
public:
  Vector3f mean; /**< The estimated pose. */
private:
  Matrix3x3f cov; /**< The covariance matrix of the estimate. */
  bool mirrored;  /**< The robot assumes to be have mirrored its pose recently, if true. */

  Vector3f sigmaPoints[7]; /**< Sigma points for updating the filter */
  Matrix3x3f l;            /**< Last computed cholesky decomposition */
  RingBufferWithSum<float, 60> validityBuffer;

public:
  float weighting;

  float validity;

  Pose2D getPose() const;

  Matrix3x3f& getCov() { return cov; };

  bool isMirrored() const { return mirrored; }

  void mirror();

  void setMirrored(bool newMirrored) { mirrored = newMirrored; }

  float computeValidity(const FieldDimensions& fieldDimensions);

  float computeValidity();

  void invalidate();

  void twist();

  void computeWeightingBasedOnValidity(const FieldDimensions& fieldDimensions, float baseValidityWeighting);

  float getVarianceWeighting() const;

  void init(const Pose2D& pose, const Pose2D& defaultPoseDeviation, float defaultValidity = 0.5f);

  void lineLocatorUpdate(const Pose2D& reading, const float covX, const float covY, const float covR);

  void motionUpdate(const Pose2D& odometryOffset,
                    const Pose2D& filterProcessDeviation,
                    const Pose2D& odometryDeviation,
                    const Vector2<>& odometryRotationDeviation);

  void performOdometryUpdate(const Pose2D& odometryOffset,
                             const Pose2D& filterProcessDeviation,
                             const Pose2D& odometryDeviation,
                             const Vector2<>& odometryRotationDeviation);

  void updateByGoalPercept(const GoalPercept& goalPercept,
                           const FieldModel& fieldModel,
                           const Vector2<>& robotRotationDeviation,
                           const CameraMatrix& cameraMatrix,
                           float goalAssociationMaxAngle,
                           float goalAssociationMaxAngularDistance);

  void updateByLineAnalysis(const LineAnalysis& lineAnalysis,
                            const FieldModel& fieldModel,
                            float centerCircleAssociationDistance,
                            float lineAssociationCorridor,
                            float cornerAssociationDistance,
                            const Vector2<>& robotRotationDeviation,
                            const FieldDimensions& fieldDimensions,
                            const CameraMatrix& cameraMatrix);

  void updateByTracker(const Vector3f& reading, const Matrix3x3f& readingCov, float ratio, bool goodCorrection);

  void computeWeightingBasedOnBallObservation(const Vector2<>& ballObservation,
                                              const Vector2<>& teamBallPosition,
                                              const float& camZ,
                                              float standardDeviationBallAngle,
                                              float standardDeviationBallDistance);

  void updateMirrorFlag(bool fallen,
                        bool armContact,
                        float maxDistanceToFieldCenterForMirrorActions,
                        const FieldDimensions& fieldDimensions);

  void draw(bool simple = false);

private:
  Matrix2x2f getCovOfPointInWorld(const Vector2<>& pointInWorld,
                                  float pointZInWorld,
                                  const CameraMatrix& cameraMatrix,
                                  const Vector2<>& robotRotationDeviation) const;

  Matrix2x2f getCovOfCircle(const Vector2<>& circlePos,
                            float centerCircleRadius,
                            const CameraMatrix& cameraMatrix,
                            const Vector2<>& robotRotationDeviation) const;

  void landmarkSensorUpdate(const Vector2<>& landmarkPosition, const Vector2f& reading, const Matrix2x2f& readingCov);

  void poseSensorUpdate(const Vector3f& reading, const Matrix3x3f& readingCov);

  void lineSensorUpdate(bool vertical, const Vector2f& reading, const Matrix2x2f& readingCov);

  void generateSigmaPoints();

  Vector2<> getOrthogonalProjection(const Vector2<>& base, const Vector2<>& dir, const Vector2<>& point) const;

  float computeAngleWeighting(float measuredAngle,
                              const Vector2<>& modelPosition,
                              const Pose2D& robotPose,
                              float standardDeviation) const;

  float computeDistanceWeighting(float measuredDistanceAsAngle,
                                 const Vector2<>& modelPosition,
                                 const Pose2D& robotPose,
                                 float cameraZ,
                                 float standardDeviation) const;
};
