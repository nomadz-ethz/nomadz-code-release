/**
 * @file BallModel.cpp
 *
 * Implementation of the BallModel's drawing functions
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</A>
 */

#include "BallModel.h"
#include "Core/System/Time.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"
#include "Core/Math/Pose2D.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Debugging/DebugDrawings3D.h"

void BallModel::draw() const {
  // drawing of the ball model in the field view
  DECLARE_DEBUG_DRAWING(
    "representation:BallModel", "drawingOnField", const Vector2<>& position(estimate.position);
    const Vector2<>& velocity(estimate.velocity);
    const Matrix2x2<>& positionCovariance(estimate.positionCovariance);
    CIRCLE("representation:BallModel",
           position.x,
           position.y,
           45,
           0, // pen width
           Drawings::ps_solid,
           ColorClasses::red,
           Drawings::bs_solid,
           ColorClasses::red);
    ARROW("representation:BallModel",
          position.x,
          position.y,
          position.x + velocity.x,
          position.y + velocity.y,
          5,
          1,
          ColorClasses::red);
    SIMPLE_COVARIANCE2D(
      "representation:BallModel", positionCovariance, position, 10, Drawings::ps_dot, ColorRGBA(255, 0, 0, 128)));

  // Declare more drawings (although they're actually drawn elsewhere)
  DECLARE_DEBUG_DRAWING("representation:BallModel:Image", "drawingOnImage");

  DECLARE_PLOT("representation:BallModel:px");
  PLOT("representation:BallModel:px", estimate.position.x);
  DECLARE_PLOT("representation:BallModel:py");
  PLOT("representation:BallModel:py", estimate.position.y);
  DECLARE_PLOT("representation:BallModel:vx");
  PLOT("representation:BallModel:vx", estimate.velocity.x);
  DECLARE_PLOT("representation:BallModel:vy");
  PLOT("representation:BallModel:vy", estimate.velocity.y);
}

// TODO Put something like this in Geometry instead!
static bool ballRadiusAtFieldPosition(const Vector2<>& centerOnField,
                                      const CameraMatrix& cameraMatrix,
                                      const CameraInfo& cameraInfo,
                                      int& ballRadius) {
  const float radius = 50.f; // mm

  // toLeft points from ball center to "left side" of ball, as seen from robot
  Vector2<> toLeft = centerOnField;
  toLeft = toLeft.rotateLeft() * (radius / centerOnField.abs());

  Vector3<> centerInSpace = {centerOnField.x, centerOnField.y, radius};
  Vector3<> leftInSpace = centerInSpace + Vector3<>(toLeft.x, toLeft.y, 0.f);
  Vector3<> rightInSpace = centerInSpace + Vector3<>(-toLeft.x, -toLeft.y, 0.f);
  Vector2<> leftInImage;
  Vector2<> rightInImage;
  bool isValid = Geometry::calculatePointInImage(leftInSpace, cameraMatrix, cameraInfo, leftInImage) &&
                 Geometry::calculatePointInImage(rightInSpace, cameraMatrix, cameraInfo, rightInImage);

  ballRadius = (int)(0.5f * (leftInImage - rightInImage).abs());
  return isValid;
}

// (x, y): projection of field location on image
// r: radius of ball in image
void BallModel::drawImage(const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo) const {
  if (estimate.position.squareAbs() == 0) {
    return;
  }

  const int maxR = 200;

  Vector2<int> bottomInImage;
  int r;
  if (Geometry::calculatePointInImage(estimate.position, cameraMatrix, cameraInfo, bottomInImage) &&
      ballRadiusAtFieldPosition(estimate.position, cameraMatrix, cameraInfo, r) && r < maxR) {
    const int x = bottomInImage.x;
    const int y = bottomInImage.y;

    ColorRGBA borderColor;
    if (seenFraction < 0.5f) {
      // Between red and yellow
      const float f = std::max(0.f, seenFraction * 2.f); // 0 to 1
      borderColor = ColorRGBA(255, (unsigned char)(255.f * f), 0);
    } else {
      // Between yellow and green
      const float f = std::min(1.f, seenFraction * 2.f - 1.f); // 0 to 1
      borderColor = ColorRGBA((unsigned char)(255.f * (1 - f)), 255, 0);
    }

    const int penWidth = 1;
    CIRCLE("representation:BallModel:Image",
           x,
           y - r,
           r,
           penWidth,
           Drawings::ps_solid,
           borderColor,
           Drawings::bs_solid,
           ColorRGBA(192, 192, 64, 90));

    Vector2<int> futureBottomInImage;
    int r2;
    if (estimate.velocity.squareAbs() > 0 &&
        Geometry::calculatePointInImage(
          estimate.position + estimate.velocity, cameraMatrix, cameraInfo, futureBottomInImage) &&
        ballRadiusAtFieldPosition(estimate.position + estimate.velocity, cameraMatrix, cameraInfo, r2)) {
      const int x2 = futureBottomInImage.x;
      const int y2 = futureBottomInImage.y;

      ARROW(
        "representation:BallModel:Image", x, y - r, x2, y2 - r2, penWidth, Drawings::ps_solid, ColorRGBA(255, 128, 0, 192));
    }
  }
}

void BallModel::draw3D(const Pose2D& robotPose) const {
  // drawing og the ball model in the scene
  DECLARE_DEBUG_DRAWING3D("representation:BallModel", "field", {
    if (Time::getTimeSince(timeWhenLastSeen) < 5000 && Time::getTimeSince(timeWhenDisappeared) < 1000) {
      Vector2<> ballRelToWorld = robotPose * estimate.position;
      SPHERE3D("representation:BallModel", ballRelToWorld.x, ballRelToWorld.y, 35.f, 35.f, ColorClasses::orange);
      LINE3D("representation:BallModel",
             robotPose.translation.x,
             robotPose.translation.y,
             1.f,
             ballRelToWorld.x,
             ballRelToWorld.y,
             1.f,
             5.f,
             ColorClasses::orange);
    }
  });
}

void BallModel::drawEndPosition(float ballFriction) const {
  // drawing of the end position
  DECLARE_DEBUG_DRAWING(
    "representation:BallModel:endPosition", "drawingOnField", Vector2<> position = estimate.getEndPosition(ballFriction);
    CIRCLE("representation:BallModel:endPosition",
           position.x,
           position.y,
           45,
           0, // pen width
           Drawings::ps_solid,
           ColorClasses::black,
           Drawings::bs_solid,
           ColorRGBA(168, 25, 99, 220)););
}

void GroundTruthBallModel::draw() const {
  DECLARE_DEBUG_DRAWING(
    "representation:GroundTruthBallModel", "drawingOnField", const Vector2<>& position(estimate.position);
    const Vector2<>& velocity(estimate.velocity);
    CIRCLE("representation:GroundTruthBallModel",
           position.x,
           position.y,
           45,
           0, // pen width
           Drawings::ps_solid,
           ColorRGBA(255, 128, 0, 192),
           Drawings::bs_solid,
           ColorRGBA(255, 128, 0, 192));
    ARROW("representation:GroundTruthBallModel",
          position.x,
          position.y,
          position.x + velocity.x,
          position.y + velocity.y,
          5,
          1,
          ColorRGBA(255, 128, 0, 192)););
}

BallModelCompressed::BallModelCompressed(const BallModel& ballModel)
    : lastPerception(ballModel.lastPerception), position(ballModel.estimate.position),
      timeWhenLastSeen(ballModel.timeWhenLastSeen), seenFraction(ballModel.seenFraction) {}

BallModelCompressed::operator BallModel() const {
  BallModel ballModel;
  ballModel.lastPerception = Vector2<>(lastPerception);
  ballModel.estimate.position = Vector2<>(position);
  // NOTE: velocity and covariance are not transferred!
  ballModel.timeWhenLastSeen = timeWhenLastSeen;
  ballModel.timeWhenDisappeared = timeWhenLastSeen; // set these equal to save space
  ballModel.timeSinceLastSeen = Time::getTimeSince(timeWhenLastSeen);
  ballModel.seenFraction = seenFraction;
  return ballModel;
}
