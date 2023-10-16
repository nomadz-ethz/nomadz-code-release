/**
 * @file FieldModel.cpp
 *
 * This file implements a submodule that represents the robot's environment for self-localization
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a> and Colin Graf
 */

#include "FieldModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"

FieldModel::FieldModel(const FieldDimensions& fieldDimensions, const CameraMatrix& cameraMatrix)
    : cameraMatrix(cameraMatrix) {
  // Initialize goal posts
  goalPosts[0] = Vector2<>(fieldDimensions.xPosOwnGoalPost, fieldDimensions.yPosRightGoal);
  goalPosts[1] = Vector2<>(fieldDimensions.xPosOwnGoalPost, fieldDimensions.yPosLeftGoal);
  goalPosts[2] = Vector2<>(fieldDimensions.xPosOpponentGoalPost, fieldDimensions.yPosLeftGoal);
  goalPosts[3] = Vector2<>(fieldDimensions.xPosOpponentGoalPost, fieldDimensions.yPosRightGoal);

  // Initialize list of relevant field lines
  for (const auto& fieldLine : fieldDimensions.getAllFieldLines()) {
    if (!fieldLine.isPartOfCircle && fieldLine.length() > 300.f) {
      FieldLine relevantFieldLine;
      relevantFieldLine.start = fieldLine.from;
      relevantFieldLine.end = fieldLine.to;
      relevantFieldLine.dir = relevantFieldLine.end - relevantFieldLine.start;
      relevantFieldLine.dir.normalize();
      relevantFieldLine.length = fieldLine.length();
      relevantFieldLine.vertical =
        std::abs(fieldLine.rotation()) < 0.001f || std::abs(normalize(fieldLine.rotation() - pi)) < 0.001f;
      fieldLines.push_back(relevantFieldLine);
    }
  }

  // Initialize corner lists:
  // X
  xCorners.push_back(Vector2<>(fieldDimensions.xPosHalfWayLine, fieldDimensions.centerCircleRadius));
  xCorners.push_back(Vector2<>(fieldDimensions.xPosHalfWayLine, -fieldDimensions.centerCircleRadius));
  // T
  tCorners = xCorners;
  tCorners.push_back(Vector2<>(fieldDimensions.xPosHalfWayLine, fieldDimensions.yPosRightSideline));
  tCorners.push_back(Vector2<>(fieldDimensions.xPosHalfWayLine, fieldDimensions.yPosLeftSideline));
  tCorners.push_back(Vector2<>(fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosLeftPenaltyArea));
  tCorners.push_back(Vector2<>(fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosRightPenaltyArea));
  tCorners.push_back(Vector2<>(fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosLeftGoalBox));
  tCorners.push_back(Vector2<>(fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosRightGoalBox));
  tCorners.push_back(Vector2<>(fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosLeftPenaltyArea));
  tCorners.push_back(Vector2<>(fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosRightPenaltyArea));
  tCorners.push_back(Vector2<>(fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosLeftGoalBox));
  tCorners.push_back(Vector2<>(fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosRightGoalBox));
  // L
  lCorners = tCorners;
  lCorners.push_back(Vector2<>(fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosRightSideline));
  lCorners.push_back(Vector2<>(fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosLeftSideline));
  lCorners.push_back(Vector2<>(fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosRightSideline));
  lCorners.push_back(Vector2<>(fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosLeftSideline));
  lCorners.push_back(Vector2<>(fieldDimensions.xPosOwnPenaltyArea, fieldDimensions.yPosRightPenaltyArea));
  lCorners.push_back(Vector2<>(fieldDimensions.xPosOwnPenaltyArea, fieldDimensions.yPosLeftPenaltyArea));
  lCorners.push_back(Vector2<>(fieldDimensions.xPosOwnGoalBox, fieldDimensions.yPosRightGoalBox));
  lCorners.push_back(Vector2<>(fieldDimensions.xPosOwnGoalBox, fieldDimensions.yPosLeftGoalBox));
  lCorners.push_back(Vector2<>(fieldDimensions.xPosOpponentPenaltyArea, fieldDimensions.yPosRightPenaltyArea));
  lCorners.push_back(Vector2<>(fieldDimensions.xPosOpponentPenaltyArea, fieldDimensions.yPosLeftPenaltyArea));
  lCorners.push_back(Vector2<>(fieldDimensions.xPosOpponentGoalBox, fieldDimensions.yPosRightGoalBox));
  lCorners.push_back(Vector2<>(fieldDimensions.xPosOpponentGoalBox, fieldDimensions.yPosLeftGoalBox));
}

bool FieldModel::getAssociatedUnknownGoalPost(const Pose2D& robotPose,
                                              const Vector2<>& goalPercept,
                                              Vector2<>& associatedPost,
                                              float goalAssociationMaxAngle,
                                              float goalAssociationMaxAngularDistance) const {
  const Vector2<> postInWorld = robotPose * goalPercept;
  if (postInWorld.x <= 0.f) // own half
  {
    if (postInWorld.y <= 0.f) { // right post
      associatedPost = goalPosts[0];
    } else { // left post
      associatedPost = goalPosts[1];
    }
  } else // opponent half
  {
    if (postInWorld.y > 0.f) { // left post
      associatedPost = goalPosts[2];
    } else { // right post
      associatedPost = goalPosts[3];
    }
  }
  return goalPostIsValid(postInWorld, associatedPost, robotPose, goalAssociationMaxAngle, goalAssociationMaxAngularDistance);
}

bool FieldModel::getAssociatedKnownGoalPost(const Pose2D& robotPose,
                                            const Vector2<>& goalPercept,
                                            bool isLeft,
                                            Vector2<>& associatedPost,
                                            float goalAssociationMaxAngle,
                                            float goalAssociationMaxAngularDistance) const {
  const Vector2<> postInWorld = robotPose * goalPercept;
  if (postInWorld.x <= 0.f) // own half
  {
    if (isLeft) { // right post [!]
      associatedPost = goalPosts[0];
    } else { // left post
      associatedPost = goalPosts[1];
    }
  } else // opponent half
  {
    if (isLeft) { // left post
      associatedPost = goalPosts[2];
    } else { // right post
      associatedPost = goalPosts[3];
    }
  }
  return goalPostIsValid(postInWorld, associatedPost, robotPose, goalAssociationMaxAngle, goalAssociationMaxAngularDistance);
}

int FieldModel::getIndexOfAssociatedLine(const Pose2D& robotPose,
                                         const Vector2<>& start,
                                         const Vector2<>& end,
                                         float lineAssociationCorridor) const {
  Vector2<> startOnField = robotPose * start;
  Vector2<> endOnField = robotPose * end;
  Vector2<> dirOnField = endOnField - startOnField;
  dirOnField.normalize();
  Vector2<> orthogonalOnField(dirOnField.y, -dirOnField.x);
  float sqrLineAssociationCorridor = sqr(lineAssociationCorridor);
  Vector2<> intersection, orthogonalProjection;

  int index = -1;
  for (unsigned int i = 0; i < fieldLines.size(); ++i) {
    const FieldLine& fieldLine = fieldLines[i];
    if (getSqrDistanceToLine(fieldLine.start, fieldLine.dir, fieldLine.length, startOnField) > sqrLineAssociationCorridor ||
        getSqrDistanceToLine(fieldLine.start, fieldLine.dir, fieldLine.length, endOnField) > sqrLineAssociationCorridor) {
      continue;
    }
    if (!intersectLineWithLine(startOnField, orthogonalOnField, fieldLine.start, fieldLine.dir, intersection)) {
      continue;
    }
    if (getSqrDistanceToLine(startOnField, dirOnField, intersection) > sqrLineAssociationCorridor) {
      continue;
    }
    if (!intersectLineWithLine(endOnField, orthogonalOnField, fieldLine.start, fieldLine.dir, intersection)) {
      continue;
    }
    if (getSqrDistanceToLine(startOnField, dirOnField, intersection) > sqrLineAssociationCorridor) {
      continue;
    }
    if (index != -1) // ambiguous?
    {
      index = -1;
      break;
    }
    index = i;
  }
  return index;
}

bool FieldModel::getAssociatedCorner(const Pose2D& robotPose,
                                     const LineAnalysis::Intersection& intersection,
                                     Vector2<>& associatedCorner,
                                     float cornerAssociationDistance) const {
  const std::vector<Vector2<>>* corners = &lCorners;
  if (intersection.type == LineAnalysis::Intersection::T) {
    corners = &tCorners;
  } else if (intersection.type == LineAnalysis::Intersection::X) {
    corners = &xCorners;
  }
  const Vector2<> pointWorld = robotPose * intersection.pos;
  const float sqrThresh = cornerAssociationDistance * cornerAssociationDistance;
  for (unsigned int i = 0; i < corners->size(); ++i) {
    const Vector2<>& c = corners->at(i);
    // simple implementation for testing:
    if ((pointWorld - c).squareAbs() < sqrThresh) {
      associatedCorner = c;
      return true;
    }
  }
  return false;
}

float FieldModel::getSqrDistanceToLine(const Vector2<>& base,
                                       const Vector2<>& dir,
                                       float length,
                                       const Vector2<>& point) const {
  float l = (point.x - base.x) * dir.x + (point.y - base.y) * dir.y;
  if (l < 0) {
    l = 0;
  }
  if (l > length) {
    l = length;
  }
  return ((base + dir * l) - point).squareAbs();
}

bool FieldModel::intersectLineWithLine(const Vector2<>& lineBase1,
                                       const Vector2<>& lineDir1,
                                       const Vector2<>& lineBase2,
                                       const Vector2<>& lineDir2,
                                       Vector2<>& intersection) const {
  float h = lineDir1.x * lineDir2.y - lineDir1.y * lineDir2.x;
  if (h == 0.f) {
    return false;
  }
  float scale = ((lineBase2.x - lineBase1.x) * lineDir1.y - (lineBase2.y - lineBase1.y) * lineDir1.x) / h;
  intersection.x = lineBase2.x + lineDir2.x * scale;
  intersection.y = lineBase2.y + lineDir2.y * scale;
  return true;
}

float FieldModel::getSqrDistanceToLine(const Vector2<>& base, const Vector2<>& dir, const Vector2<>& point) const {
  float l = (point.x - base.x) * dir.x + (point.y - base.y) * dir.y;
  return ((base + dir * l) - point).squareAbs();
}

bool FieldModel::goalPostIsValid(const Vector2<>& observedPosition,
                                 const Vector2<>& modelPosition,
                                 const Pose2D& robotPose,
                                 float goalAssociationMaxAngle,
                                 float goalAssociationMaxAngularDistance) const {
  const float modelAngle = Geometry::angleTo(robotPose, modelPosition);
  const float observedAngle = Geometry::angleTo(robotPose, observedPosition);
  if (std::abs(normalize(modelAngle - observedAngle)) > goalAssociationMaxAngle) {
    return false;
  }
  const float modelDistance = (robotPose.translation - modelPosition).abs();
  const float modelDistanceAsAngle = (pi_2 - std::atan2(cameraMatrix.translation.z, modelDistance));
  const float observedDistance = (robotPose.translation - observedPosition).abs();
  const float observedDistanceAsAngle = (pi_2 - std::atan2(cameraMatrix.translation.z, observedDistance));
  return std::abs(modelDistanceAsAngle - observedDistanceAsAngle) < goalAssociationMaxAngularDistance;
}
