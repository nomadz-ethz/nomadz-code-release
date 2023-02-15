/**
 * @file FieldModel.h
 *
 * This file declares a submodule that represents the robot's environment for self-localization
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a> and Colin Graf
 */

#pragma once

#include "Representations/Perception/LineAnalysis.h"
#include "Core/Math/Vector2.h"
#include <vector>

class FieldDimensions;
class Pose2D;
class CameraMatrix;
class RobotPose;

/**
 * @class FieldModel
 *
 * A class for mapping observations of a
 * certain class to the closest positions in the field model.
 */
class FieldModel {
public:
  /**
   * A field line relative to the robot.
   */
  class FieldLine {
  public:
    Vector2<> start; /**< The starting point of the line. */
    Vector2<> end;   /**< The ending point of the line. */
    Vector2<> dir;   /**< The normalized direction of the line (from starting point). */
    float length;    /**< The length of the line. */
    bool vertical;   /**< Whether this is a vertical or horizontal line. */
  };

  std::vector<FieldLine> fieldLines; /**< Relevant field lines  */

private:
  Vector2<> goalPosts[4]; /**< The positions of the goal posts. */
  const CameraMatrix& cameraMatrix;
  std::vector<Vector2<>> xCorners;
  std::vector<Vector2<>> lCorners;
  std::vector<Vector2<>> tCorners;

public:
  /**
   * Constructor, initialized the field model
   */
  FieldModel(const FieldDimensions& fieldDimensions, const CameraMatrix& cameraMatrix);

  bool getAssociatedUnknownGoalPost(const Pose2D& robotPose,
                                    const Vector2<>& goalPercept,
                                    Vector2<>& associatedPost,
                                    float goalAssociationMaxAngle,
                                    float goalAssociationMaxAngularDistance) const;

  bool getAssociatedKnownGoalPost(const Pose2D& robotPose,
                                  const Vector2<>& goalPercept,
                                  bool isLeft,
                                  Vector2<>& associatedPost,
                                  float goalAssociationMaxAngle,
                                  float goalAssociationMaxAngularDistance) const;

  bool getAssociatedCorner(const Pose2D& robotPose,
                           const LineAnalysis::Intersection& intersection,
                           Vector2<>& associatedCorner,
                           float cornerAssociationDistance) const;

  int getIndexOfAssociatedLine(const Pose2D& robotPose,
                               const Vector2<>& start,
                               const Vector2<>& end,
                               float lineAssociationCorridor) const;

private:
  float getSqrDistanceToLine(const Vector2<>& base, const Vector2<>& dir, float length, const Vector2<>& point) const;

  float getSqrDistanceToLine(const Vector2<>& base, const Vector2<>& dir, const Vector2<>& point) const;

  bool intersectLineWithLine(const Vector2<>& lineBase1,
                             const Vector2<>& lineDir1,
                             const Vector2<>& lineBase2,
                             const Vector2<>& lineDir2,
                             Vector2<>& intersection) const;

  bool goalPostIsValid(const Vector2<>& observedPosition,
                       const Vector2<>& modelPosition,
                       const Pose2D& robotPose,
                       float goalAssociationMaxAngle,
                       float goalAssociationMaxAngularDistance) const;
};
