/**
 * @file GroundTruthProvider.cpp
 *
 * Imports ground truth data from SimRobot, then simulates providing various representations based on that data.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "GroundTruthProvider.h"
#include "Core/Range.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"

void GroundTruthProvider::update(GroundTruthBallModel& groundTruthBallModel) {
  importGroundTruth();
  groundTruthBallModel = this->groundTruthBallModel;
}

void GroundTruthProvider::update(GroundTruthPlayerModel& groundTruthPlayerModel) {
  importGroundTruth();
  groundTruthPlayerModel = this->groundTruthPlayerModel;
}

void GroundTruthProvider::update(GroundTruthRobotPose& groundTruthRobotPose) {
  importGroundTruth();
  groundTruthRobotPose = this->groundTruthRobotPose;
}

void GroundTruthProvider::update(BallModel& ballModel) {
  importGroundTruth();
  ballModel = groundTruthBallModel;
  ballModel.drawImage(theCameraMatrix, theCameraInfo);
}

void GroundTruthProvider::update(LinePercept& linePercept) {
  importGroundTruth();

  const Pose2D poseInv = groundTruthRobotPose.invert();
  const auto cameraMatrixInv = theCameraMatrix.invert();
  const int lineMinLength2 = theGroundTruthParams.lineMinLength * theGroundTruthParams.lineMinLength;

  linePercept.fieldLineSegments.clear();

  for (const auto& line : theFieldDimensions.getAllFieldLines()) {
    // Line endpoints in global coords
    const Vector2<> glo1 = line.from;
    const Vector2<> glo2 = line.to;

    // Line endpoints local to groundTruthRobotPose
    Vector2<> loc1 = poseInv * glo1;
    Vector2<> loc2 = poseInv * glo2;

    // If limiting line perception range
    if (theGroundTruthParams.lineMaxRange >= 0.f) {
      const Geometry::Circle circle = {{0.f, 0.f}, theGroundTruthParams.lineMaxRange};
      if (!Geometry::clipLineWithCircle(circle, loc1, loc2)) {
        continue;
      }
    }

    // Line endpoints in camera image, skipping those entirely outside of the image
    Vector2<int> img1, img2;
    if (!Geometry::calculateLineInImage(loc1, loc2, cameraMatrixInv, theCameraInfo, img1, img2)) {
      continue;
    }
    if (!Geometry::clipLineWithRectangle({0, 0}, {theCameraInfo.width, theCameraInfo.height}, img1, img2)) {
      continue;
    }

    // Simulate ShapePerceptor's lower resolution (scale down, floor, then scale up)
    if (theGroundTruthParams.lessResolution) {
      lowerResolution(img1);
      lowerResolution(img2);
    }

    // Skip lines too small to be seen
    if ((img1 - img2).squareAbs() < lineMinLength2) {
      continue;
    }

    // Turn into ShapePercept::LineSegment
    ShapePercept::LineSegment segment;
    segment.p1 = img1;
    segment.p2 = img2;
    segment.votes = 100; // arbitrary
    segment.hits = 100;  // arbitrary

    linePercept.fieldLineSegments.push_back(segment);
  }
}

// Out of all penalty marks within the image & within sight range of robot, provide the nearest one
void GroundTruthProvider::update(PenaltyMarkPercept& penaltyMarkPercept) {

  const Vector2<> glo1 = {theFieldDimensions.xPosOwnPenaltyMark, 0.f};
  const Vector2<> glo2 = {theFieldDimensions.xPosOpponentPenaltyMark, 0.f};
  const float penaltyMarkMaxRange2 = theGroundTruthParams.penaltyMarkMaxRange * theGroundTruthParams.penaltyMarkMaxRange;

  const Vector2<int> topLeft = {0, 0};
  const Vector2<int> bottomRight = {theCameraInfo.width, theCameraInfo.height};

  const auto poseInv = groundTruthRobotPose.invert();
  const Vector2<> loc1 = poseInv * glo1;
  const Vector2<> loc2 = poseInv * glo2;

  Vector2<int> img1, img2;
  const bool visible1 = loc1.squareAbs() < penaltyMarkMaxRange2 &&
                        Geometry::calculatePointInImage(loc1, theCameraMatrix, theCameraInfo, img1) &&
                        Geometry::isPointInsideRectangle(topLeft, bottomRight, img1);
  const bool visible2 = loc2.squareAbs() < penaltyMarkMaxRange2 &&
                        Geometry::calculatePointInImage(loc2, theCameraMatrix, theCameraInfo, img2) &&
                        Geometry::isPointInsideRectangle(topLeft, bottomRight, img2);

  penaltyMarkPercept.seen = visible1 || visible2;

  if (visible1 || visible2) {
    penaltyMarkPercept.seen = true;

    // Choose 1 only if it is closer than 2 or 2 is not visible
    bool choose1;
    if (visible1 && visible2) {
      choose1 = loc1.squareAbs() < loc2.squareAbs();
    } else {
      choose1 = visible1;
    }

    Vector2<int> imgPos = choose1 ? img1 : img2;
    if (theGroundTruthParams.lessResolution) {
      lowerResolution(imgPos);
    }

    // Calculate relativePositionOnField
    penaltyMarkPercept.positionInImage = imgPos;
    Geometry::calculatePointOnField(
      penaltyMarkPercept.positionInImage, theCameraMatrix, theCameraInfo, penaltyMarkPercept.relativePositionOnField);

  } else {
    penaltyMarkPercept.seen = false;
  }
}

void GroundTruthProvider::update(PlayerModel& playerModel) {
  importGroundTruth();
  playerModel = groundTruthPlayerModel;
}

void GroundTruthProvider::update(PlayerPercept& playerPercept) {
  importGroundTruth();

  DECLARE_DEBUG_DRAWING("module:GroundTruthProvider:Field", "drawingOnField");

  playerPercept.players.clear();
  for (const auto& groundTruthPlayer : groundTruthPlayerModel.players) {
    PlayerPercept::Player player;

    {
      Vector2<int> imagePos;
      bool canSee = false;

      auto left = groundTruthPlayer.relPosOnField +
                  groundTruthPlayer.relPosOnField.left().direction() * theGroundTruthParams.playerRadius;
      auto right = groundTruthPlayer.relPosOnField +
                   groundTruthPlayer.relPosOnField.right().direction() * theGroundTruthParams.playerRadius;
      LINE("module:GroundTruthProvider:Field",
           groundTruthPlayer.relPosOnField.x,
           groundTruthPlayer.relPosOnField.y,
           left.x,
           left.y,
           30,
           Drawings::ps_solid,
           ColorClasses::red);
      LINE("module:GroundTruthProvider:Field",
           groundTruthPlayer.relPosOnField.x,
           groundTruthPlayer.relPosOnField.y,
           right.x,
           right.y,
           30,
           Drawings::ps_solid,
           ColorClasses::blue);

      if (Geometry::calculatePointInImage(groundTruthPlayer.relPosOnField +
                                            groundTruthPlayer.relPosOnField.left().direction() *
                                              theGroundTruthParams.playerRadius,
                                          theCameraMatrix,
                                          theCameraInfo,
                                          imagePos)) {
        player.x1 = Range<int>(0, theCameraInfo.width).limit(imagePos.x);
        canSee |= (imagePos.x >= 0) && (imagePos.x < theCameraInfo.width);
      }
      if (Geometry::calculatePointInImage(groundTruthPlayer.relPosOnField +
                                            groundTruthPlayer.relPosOnField.right().direction() *
                                              theGroundTruthParams.playerRadius,
                                          theCameraMatrix,
                                          theCameraInfo,
                                          imagePos)) {
        player.x2 = Range<int>(0, theCameraInfo.width).limit(imagePos.x);
        canSee |= (imagePos.x >= 0) && (imagePos.x < theCameraInfo.width);
      }
      if (Geometry::calculatePointInImage(groundTruthPlayer.relPosOnField, theCameraMatrix, theCameraInfo, imagePos)) {
        player.y2 = Range<int>(0, theCameraInfo.height).limit(imagePos.y);
        player.jerseyY1 = player.y2;
      }
      if (Geometry::calculatePointInImage(Vector3<>(float(groundTruthPlayer.relPosOnField.x),
                                                    float(groundTruthPlayer.relPosOnField.y),
                                                    theGroundTruthParams.playerHeight),
                                          theCameraMatrix,
                                          theCameraInfo,
                                          imagePos)) {
        player.y1 = Range<int>(0, theCameraInfo.height).limit(imagePos.y);
        player.jerseyY0 = player.y1;
      }

      if (!canSee) {
        continue;
      }
    }

    player.centerOnField = groundTruthPlayer.relPosOnField;
    player.radius = theGroundTruthParams.playerRadius;
    player.detectedJersey = true;
    player.opponent = groundTruthPlayer.opponent;
    player.standing = groundTruthPlayer.standing;

    playerPercept.players.push_back(player);
  }
}

void GroundTruthProvider::update(RobotPose& robotPose) {
  importGroundTruth();
  MODIFY("parameters:GroundTruth", theGroundTruthParams);
  robotPose = groundTruthRobotPose;

  // Only to get the origin:RobotPose drawing
  EXECUTE_ONLY_IN_DEBUG(robotPose.draw(true););
}

void GroundTruthProvider::importGroundTruth() {
  if (theGroundTruthWorldState.time == lastGroundTruthTime) {
    return;
  }

  lastGroundTruthTime = theGroundTruthWorldState.time;

  // Ball
  {
    auto& ball = groundTruthBallModel;

    if (theGroundTruthWorldState.balls.empty()) {
      ball.timeSinceLastSeen = theFrameInfo.time - ball.timeWhenLastSeen;
      ball.seenFraction = 0.f;

    } else {
      const Vector2<> globalPos(theGroundTruthWorldState.balls[0].x, theGroundTruthWorldState.balls[0].y);
      const Vector2<> localPos = theGroundTruthWorldState.ownPose.invert() * globalPos;
      Vector2<int> imagePos;
      if (Geometry::calculatePointInImage(localPos, theCameraMatrix, theCameraInfo, imagePos)) {
        ball.lastPerception = Vector2<>(imagePos);
      }

      ball.estimate.position = localPos;
      const Vector2<> globalVel =
        (globalPos - lastGlobalBallPos) / float(theFrameInfo.getTimeSince(ball.timeWhenLastSeen) / 1000.f);
      const Vector2<> localVel = globalVel.rotated(-theGroundTruthWorldState.ownPose.rotation);
      ball.estimate.velocity = localVel;
      ball.estimate.positionCovariance = Matrix2x2<>();
      if (!theGroundTruthParams.disableBallValidity) {
        ball.timeWhenLastSeen = theFrameInfo.time;
        ball.timeWhenDisappeared = 0;
        ball.timeSinceLastSeen = 0;
      } else {
        ball.timeWhenLastSeen = 0;
        ball.timeWhenDisappeared = 0;
        ball.timeSinceLastSeen = 200000;
      }
      ball.seenFraction = 1.f;

      lastGlobalBallPos = globalPos;
    }
  }

  // Players
  {
    auto& players = groundTruthPlayerModel.players;
    players.clear();

    // Teammates
    for (const GroundTruthWorldState::Player& teammate : theGroundTruthWorldState.teammates) {
      PlayerModel::Player player;
      player.relPosOnField = theGroundTruthWorldState.ownPose.invert() * teammate.pose.translation;
      player.opponent = false;
      player.standing = teammate.upright;
      player.covariance = {1.f, 0.f, 0.f, 1.f};
      player.minVariance = 1.f;
      player.timeStamp = theFrameInfo.time;
      player.firstSeen = 0;
      player.detected = true;
      players.push_back(player);
    }

    // Opponents
    for (const GroundTruthWorldState::Player& opponent : theGroundTruthWorldState.opponents) {
      PlayerModel::Player player;
      player.relPosOnField = theGroundTruthWorldState.ownPose.invert() * opponent.pose.translation;
      player.opponent = true;
      player.standing = opponent.upright;
      player.covariance = {1.f, 0.f, 0.f, 1.f};
      player.minVariance = 1.f;
      player.timeStamp = theFrameInfo.time;
      player.firstSeen = 0;
      player.detected = true;
      players.push_back(player);
    }
  }

  // Own pose
  {
    auto& pose = groundTruthRobotPose;

    (Pose2D&)pose = theGroundTruthWorldState.ownPose;
    pose.deviation = 1.f;
    pose.covariance = Matrix3x3<>();
    if (!theGroundTruthParams.disableRobotPoseValidity) {
      pose.validity = 1.f; // (0 = invalid, 1 = perfect)
      pose.timeSinceLastValid = 0;
      pose.timeWhenLastValid = theFrameInfo.time;
    } else {
      pose.validity = 0.f;
      pose.timeSinceLastValid = 100000;
      pose.timeWhenLastValid = 0;
    }
  }
}

void GroundTruthProvider::lowerResolution(Vector2<int>& p) {
  p = (p / 2) * 2;
}

MAKE_MODULE(GroundTruthProvider, Modeling)
