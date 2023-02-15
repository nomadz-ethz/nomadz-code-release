/**
 * @file LineLocator.cpp
 *
 * Looks for interesting field line features & offers hypotheses of poses
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "LineLocator.h"
#include "Core/ColorClasses.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Debugging/DebugDrawings3D.h"
#include "Core/Debugging/DebugImages.h"
#include "Core/Debugging/Debugging.h"
#include "Core/Math/Vector2.h"
#include "Core/Math/Common.h"

static inline Pose2D mirrorPose(const Pose2D& pose) {
  if (pose.rotation >= 0) {
    return Pose2D(pose.rotation - (float)M_PI, -pose.translation.x, -pose.translation.y);
  } else {
    return Pose2D(pose.rotation + (float)M_PI, -pose.translation.x, -pose.translation.y);
  }
}

// Basically find pose so that (refAng, refPos) would be seen at (localAng, localPos)
static Pose2D
poseFromReference(const float localAng, const float refAng, const Vector2<>& localPos, const Vector2<>& refPos) {
  Pose2D pose;
  pose.rotation = wrapAngle(refAng - localAng);

  const float ang = localPos.angle() + pose.rotation + (float)M_PI;
  const float dist = localPos.abs();

  pose.translation.x = dist * cosf(ang);
  pose.translation.y = dist * sinf(ang);
  pose.translation += refPos;

  return pose;
}

static Pose2D averagePose(const Pose2D& pose1, const Pose2D& pose2) {
  Pose2D pose;
  if (std::abs(pose1.rotation - pose2.rotation) <= (float)M_PI) {
    pose.rotation = (pose1.rotation + pose2.rotation) / 2;
  } else {
    // Mean angle is on "other side" of the circle from 0, e.g. with -177 and 178 degrees
    pose.rotation = wrapAngle((pose1.rotation + pose2.rotation) / 2 + (float)M_PI);
  }
  pose.translation.x = (pose1.translation.x + pose2.translation.x) / 2;
  pose.translation.y = (pose1.translation.y + pose2.translation.y) / 2;
  return pose;
}

static bool collinear(const Vector2<>& dir1, const Vector2<>& dir2, float threshold) {
  return fabsf(wrapAngle(dir1.angle() - dir2.angle())) < threshold ||
         fabsf(wrapAngle(dir1.angle() - dir2.angle() - (float)M_PI)) < threshold;
}

void LineLocator::init() {}

void LineLocator::drawPose(const Pose2D& pose, ColorRGBA color = ColorRGBA(255, 0, 0)) {
  Vector2<> bodyPoints[4] = {Vector2<>(55, 90), Vector2<>(-55, 90), Vector2<>(-55, -90), Vector2<>(55, -90)};
  for (int i = 0; i < 4; i++) {
    bodyPoints[i] = pose * bodyPoints[i];
  }
  Vector2<> dirVec(200, 0);
  dirVec = pose * dirVec;
  LINE("module:LineLocator:poses",
       pose.translation.x,
       pose.translation.y,
       dirVec.x,
       dirVec.y,
       20,
       Drawings::ps_solid,
       ColorClasses::white);
  POLYGON("module:LineLocator:poses", 4, bodyPoints, 20, Drawings::ps_solid, color, Drawings::bs_solid, ColorClasses::white);
  CIRCLE("module:LineLocator:poses",
         pose.translation.x,
         pose.translation.y,
         42,
         0,
         Drawings::ps_solid,
         color,
         Drawings::bs_solid,
         color);

  DECLARE_DEBUG_DRAWING3D("module:LineLocator:poses", "field", {
    LINE3D("module:LineLocator:poses", pose.translation.x, pose.translation.y, 10, dirVec.x, dirVec.y, 10, 1, color);
    for (int i = 0; i < 4; ++i) {
      const Vector2<> p1 = bodyPoints[i];
      const Vector2<> p2 = bodyPoints[(i + 1) & 3];
      LINE3D("module:LineLocator:poses", p1.x, p1.y, 10, p2.x, p2.y, 10, 1, color);
    }
  });
}

bool LineLocator::isOutOfField(const Pose2D& pose, float margin = 0.0f) const {
  return fabsf(pose.translation.x) > theFieldDimensions.xPosOpponentGroundline + margin ||
         fabsf(pose.translation.y) > theFieldDimensions.yPosLeftSideline + margin;
}

void LineLocator::update(LineLocalization& lineLocalization) {

  // TODO: ShapePerceptor: clean up detected lines, especially "shards" that go off the side of a long line

  typedef LineLocalization::Hypothesis Hypothesis;
  typedef LineLocalization::Line Line;
  typedef LineLocalization::CircleSpot CircleSpot;
  typedef LineLocalization::Intersection Intersection;
  typedef LineLocalization::PenaltyAreaSide PenaltyAreaSide;

  const FieldDimensions& dim = theFieldDimensions;

  DECLARE_DEBUG_DRAWING("module:LineLocator:poses", "drawingOnField");

  Pose2D pose;
  std::vector<Hypothesis> hypotheses;

  lineLocalization.hypotheses.clear();

  // Always store features when in lower camera.
  // Always process features when in upper camera.
  // When we get twice in a row from the same camera, clear the previous/old information.
  if (lastCamera == theCameraInfo.camera || (lastCamera == CameraInfo::upper && theCameraInfo.camera == CameraInfo::lower)) {
    lines.clear();
    corners.clear();
    tees.clear();
    crosses.clear();
    penaltyAreaSides.clear();

    ballSeenLower = ballSeenUpper = false;
    ballPosLower = ballPosUpper = Vector2<>(0.f, 0.f);

    markSeenLower = markSeenUpper = false;
    markPosLower = markPosUpper = Vector2<>(0.f, 0.f);
  }

  const bool isKeeper = (theBehaviorControlOutput.behaviorStatus.role == BehaviorStatus::keeper);

  /* Start pre-processing & store */

  // Store lines & midLine
  bool centerFound = false;
  Line midLine;
  {
    bool circleFound = theLineAnalysis.circle.found;
    bool midLineFound = false;

    for (size_t i = 0; i < theLineAnalysis.lines.size(); ++i) {
      Line line = theLineAnalysis.lines[i];

      // Set line.first as left end, line.last as right end
      // TODO Check if this really works as expected
      if (line.last.y > line.first.y) {
        line.alpha = wrapAngle(line.alpha + (float)M_PI);
        line.d = -line.d;
        std::swap(static_cast<Vector2<>&>(line.first), static_cast<Vector2<>&>(line.last));
      }

      if (line.midLine) {
        midLineFound = true;
        midLine = line;
      } else {
        lines.push_back(line);
      }
    }

    // Find a possible midLine if not found so far
    if (!midLineFound && circleFound) {
      for (size_t i = 0; i < lines.size(); ++i) {
        // Determine if this might be an actual midLine (maybe from the lower cam)
        const Line& line = lines[i];
        const float distFromCenter = fabsf(line.calculateDistToLine(theLineAnalysis.circle.pos));
        if (distFromCenter < 50.f) {
          midLineFound = true;
          midLine = line;
          lines.erase(lines.begin() + i);
          break;
        }
      }
    }

    centerFound = circleFound && midLineFound;
    if (centerFound) {
      // Check circle is correct w.r.t midLine
      const CircleSpot& circle = theLineAnalysis.circle;

      const float edgeTol = 5.f; // px
      bool midLineCut = false;
      if (std::min<float>(midLine.startInImage.x, midLine.endInImage.x) < edgeTol ||
          std::max<float>(midLine.startInImage.x, midLine.endInImage.x) > theCameraInfo.width - edgeTol) {
        midLineCut = true;
      }
      const bool circleInBoundingBox =
        circle.pos.y <= midLine.first.y + dim.centerCircleRadius &&
        circle.pos.y >= midLine.last.y - dim.centerCircleRadius &&
        circle.pos.x >= std::min<float>(midLine.first.x, midLine.last.x) - dim.centerCircleRadius &&
        circle.pos.x <= std::max<float>(midLine.first.x, midLine.last.x) + dim.centerCircleRadius;
      const bool circleCloseToMidLine = fabsf(midLine.calculateDistToLine(circle.pos)) <= dim.centerCircleRadius;
      if ((midLineCut && !circleCloseToMidLine) || (!midLineCut && !circleInBoundingBox)) {
        // OUTPUT(idText, text, "toyUpdate: Ignoring center circle at (" << circle.pos.x << ", " << circle.pos.y << ")");
        // if (midLineCut && !circleCloseToMidLine) OUTPUT(idText, text, "Reason: circle far from midLine");
        // if (!midLineCut && !circleInBoundingBox) OUTPUT(idText, text, "Reason: circle not in midLine bounding box");
        centerFound = false;
        lines.push_back(midLine);
      }
    }
  }

  for (size_t i = 0; i < theLineAnalysis.intersections.size(); ++i) {
    Intersection intersection = theLineAnalysis.intersections[i];
    float angleDelta = wrapAngle(intersection.dir1.angle() - intersection.dir2.angle());
    switch (intersection.type) {
    case Intersection::L:
      // Set dir1 as left segment, dir2 as right segment of corner (considered in a ^ orientation)
      if (angleDelta > 0) {
        std::swap(static_cast<Vector2<>&>(intersection.dir1), static_cast<Vector2<>&>(intersection.dir2));
        std::swap(static_cast<Vector2<>&>(intersection.first1), static_cast<Vector2<>&>(intersection.first2));
        std::swap(static_cast<Vector2<>&>(intersection.last1), static_cast<Vector2<>&>(intersection.last2));
      }
      corners.push_back(intersection);
      break;
    case Intersection::T:
      // Set dir1 as bottom segment, dir2 as left segment of tee (considered in a T orientation)
      if (angleDelta > 0) {
        // Should go from left to right
        intersection.dir2 = -intersection.dir2;
        std::swap(static_cast<Vector2<>&>(intersection.first2), static_cast<Vector2<>&>(intersection.last2));
      }
      tees.push_back(intersection);
      break;
    case Intersection::X:
      crosses.push_back(intersection);
      break;
    }
  }

  // Find penalty area sides
  {
    const float areaWidth = dim.xPosOpponentGroundline - dim.xPosOpponentPenaltyArea; // mm
    const float areaLength = 2.f * dim.yPosLeftPenaltyArea;
    auto tee = tees.begin();
    while (tee != tees.end()) {
      bool foundMatch = false;
      auto corner = corners.begin();
      while (corner != corners.end()) {

        // Find if bottom of tee is pointing to corner
        const float cornerTeeDist = (corner->pos - tee->pos).abs();
        const float longLine =
          std::max<float>((corner->first1 - corner->last1).abs(), (corner->first2 - corner->last2).abs());
        if (collinear(tee->dir1, corner->pos - tee->pos, 0.01f) && cornerTeeDist > 0.8f * areaWidth &&
            cornerTeeDist < 1.25f * areaWidth && longLine > 0.4f * areaLength && longLine < 1.25f * areaLength) {
          PenaltyAreaSide side;
          side.tee = *tee;
          side.corner = *corner;

          // Find whether left or right segment of corner is pointing to tee
          const float angle1 = wrapAngle(corner->dir1.angle() - (tee->pos - corner->pos).angle());
          const float angle2 = wrapAngle(corner->dir2.angle() - (tee->pos - corner->pos).angle());
          if (fabsf(angle1) < fabsf(angle2)) {
            side.isLeft = false;
          } else {
            side.isLeft = true;
          }

          penaltyAreaSides.push_back(side);

          corner = corners.erase(corner);
          tee = tees.erase(tee);
          foundMatch = true;
          break;
        }
        if (!foundMatch) {
          ++corner;
        }
      }
      if (!foundMatch) {
        ++tee;
      }
    }
  }

  // Remove pairs of connected corners (likely misidentified penalty area sides)
  if (corners.size() >= 2) {

    for (size_t i = 0; i < corners.size(); ++i) {
      Intersection& corner1 = corners[i];

      for (size_t j = i + 1; j < corners.size(); ++j) {
        Intersection& corner2 = corners[j];

        if (collinear(corner1.dir1, corner2.pos - corner1.pos, 0.01f) ||
            collinear(corner1.dir2, corner2.pos - corner1.pos, 0.01f)) {
          corners.erase(corners.begin() + i);
          corners.erase(corners.begin() + j);
          --i;
          break;
        }
      }
    }
  }

  // Store ball if on lower camera
  if (theCameraInfo.camera == CameraInfo::lower) {
    ballSeenLower = theBallPercept.ballWasSeen;
    ballPosLower = theBallPercept.relativePositionOnField;
    markSeenLower = thePenaltyMarkPercept.seen;
    markPosLower = thePenaltyMarkPercept.relativePositionOnField;
  } else {
    ballSeenUpper = theBallPercept.ballWasSeen;
    ballPosUpper = theBallPercept.relativePositionOnField;
    markSeenUpper = thePenaltyMarkPercept.seen;
    markPosUpper = thePenaltyMarkPercept.relativePositionOnField;
  }

  // TODO Remove corners that have a line sticking out of them

  lastCamera = theCameraInfo.camera;

  // Only process features on upper camera
  if (theCameraInfo.camera == CameraInfo::lower) {
    return;
  }

  /* Start update */

  // Center circle & middle line
  {
    if (centerFound) {
      const Vector2<> midLineDir = midLine.last - midLine.first;
      const CircleSpot& circle = theLineAnalysis.circle;

      pose = poseFromReference(midLineDir.angle(), (float)M_PI / 2, circle.pos, Vector2<>(0.f, 0.f));

      drawPose(pose, ColorClasses::orange);
      drawPose(mirrorPose(pose), ColorClasses::orange);

      hypotheses.push_back(Hypothesis(pose, circle.pos.abs(), ColorClasses::orange));
      hypotheses.push_back(Hypothesis(mirrorPose(pose), circle.pos.abs(), ColorClasses::orange));
    }
  }

  // Penalty area corners
  {
    for (size_t i = 0; i < penaltyAreaSides.size(); ++i) {
      const Intersection tee = penaltyAreaSides[i].tee;
      const Intersection corner = penaltyAreaSides[i].corner;
      const bool isLeft = penaltyAreaSides[i].isLeft;

      const Vector2<> referenceCorner = isLeft ? Vector2<>(dim.xPosOpponentPenaltyArea, dim.yPosLeftPenaltyArea)
                                               : Vector2<>(dim.xPosOpponentPenaltyArea, dim.yPosRightPenaltyArea);

      pose =
        averagePose(poseFromReference(corner.dir1.angle(), (float)(isLeft ? -M_PI / 2 : 0), corner.pos, referenceCorner),
                    poseFromReference(corner.dir2.angle(), (float)(isLeft ? 0 : M_PI / 2), corner.pos, referenceCorner));

      drawPose(pose, ColorRGBA(192, 0, 192)); // Purple
      drawPose(mirrorPose(pose), ColorRGBA(192, 0, 192));

      hypotheses.push_back(Hypothesis(pose, corner.pos.abs(), ColorRGBA(192, 0, 192)));
      hypotheses.push_back(Hypothesis(mirrorPose(pose), corner.pos.abs(), ColorRGBA(192, 0, 192)));
    }
  }

  // Penalty mark + penalty area line
  {
    if (markSeenUpper || markSeenLower) {
      bool hypothesisPM = false;
      const Vector2<>& markPos = markSeenUpper ? markPosUpper : markPosLower;

      // Find penalty area line
      Line bestLine;
      float bestLineLen = 9999999999999.f; // HACK Big number
      Vector2<> bestLineClosestPoint;
      for (size_t i = 0; i < lines.size(); ++i) {
        const Line& line = lines[i];

        const float len = (line.first - line.last).abs();
        const float refLen = 2.f * dim.yPosLeftPenaltyArea; // Near side of penalty area

        const float lineMarkDist = fabsf(line.calculateDistToLine(markPos));
        const float refLineMarkDist = dim.xPosOpponentPenaltyArea - dim.xPosOpponentPenaltyMark;

        // OUTPUT(idText, text, "Found line at distance to mark " << lineMarkDist << " (" << 0.9*refLineMarkDist << " ... "
        // << 1.1*refLineMarkDist << " nominal), len " << len << " (" << 0.1*refLen << " ... " << 1.1f*refLen << " nominal),
        // distance " << fabsf(line.d) << "(0 ... 2000 nominal)");

        if (lineMarkDist > 0.9 * refLineMarkDist && lineMarkDist < 1.1 * refLineMarkDist // Line-mark distance
            && len > 0.2f * refLen && len < 1.1f * refLen                                // Line length
            /*&& line.pointIsBesideSegment(markPos)*/) {                                 // Mark is beside line segment

          if ((line.first.x - markPos.x) > 0 || (line.last.x - markPos.x) > 0) { // If facing goal or side
            for (size_t a = 0; a < lines.size(); ++a) {

              const Line& groundLine = lines[a];
              const float lenGround = (groundLine.first - groundLine.last).abs();
              const float lineLineDist = fabsf(line.calculateDistToLine(groundLine.first));
              const float lineGroundDist = dim.xPosOpponentGroundline - dim.xPosOpponentPenaltyArea;

              if (lineLineDist > 0.8 * lineGroundDist && lineLineDist < 1.2 * lineGroundDist && lenGround > 0.8 * len) {

                if (fabsf(len - refLen) < bestLineLen) {
                  bestLine = line;
                  bestLineLen = fabsf(len - refLen);

                  // Find closest point to markPos on line
                  const Vector2<> x0 = markPos - line.first;
                  const Vector2<> v = Vector2<>(line.last.x - line.first.x, line.last.y - line.first.y);
                  const Vector2<> proj = v * (x0.x * v.x + x0.y * v.y) / v.squareAbs();

                  // OUTPUT(idText, text, "line.first = " << line.first.x << ", " << line.first.y << "; line.last = " <<
                  // line.last.x << ", " << line.last.y << "; markPos = " << markPos.x << ", " << markPos.y << "; x0 = " <<
                  // x0.x << ", " << x0.y << "; v = " << v.x << ", " << v.y << "; proj = " << proj.x << ", " << proj.y << ";
                  // return " << (line.first+proj).x << ", " << (line.first+proj).y);
                  bestLineClosestPoint = line.first + proj;
                  hypothesisPM = true;
                }

                break;
              }
            }
          } else {

            bool wrong = false;
            for (size_t a = 0; a < lines.size(); ++a) {
              if (i == a) {
                continue;
              }
              // Hypothesis is not valid if another line is seen that is closer than the middle circle
              const Line& line2 = lines[a];
              const float lineLineDist =
                std::min(fabsf(line.calculateDistToLine(line2.first)), fabsf(line.calculateDistToLine(line2.last)));
              const float distToClosestPossibleLine = dim.xPosOpponentPenaltyArea - dim.centerCircleRadius;

              if (lineLineDist < distToClosestPossibleLine) {
                wrong = true;
              }
            }

            if (!wrong) {
              if (fabsf(len - refLen) < bestLineLen) {
                bestLine = line;
                bestLineLen = fabsf(len - refLen);

                // Find closest point to markPos on line
                const Vector2<> x0 = markPos - line.first;
                const Vector2<> v = Vector2<>(line.last.x - line.first.x, line.last.y - line.first.y);
                const Vector2<> proj = v * (x0.x * v.x + x0.y * v.y) / v.squareAbs();

                // OUTPUT(idText, text, "line.first = " << line.first.x << ", " << line.first.y << "; line.last = " <<
                // line.last.x << ", " << line.last.y << "; markPos = " << markPos.x << ", " << markPos.y << "; x0 = " <<
                // x0.x << ", " << x0.y << "; v = " << v.x << ", " << v.y << "; proj = " << proj.x << ", " << proj.y << ";
                // return " << (line.first+proj).x << ", " << (line.first+proj).y);
                bestLineClosestPoint = line.first + proj;
                hypothesisPM = true;
              }
            }
          }
        }
      }

      if (hypothesisPM) {
        const Vector2<> refMark = Vector2<>(dim.xPosOwnPenaltyMark, 0.f);

        pose = poseFromReference((bestLineClosestPoint - markPos).angle(), -(float)M_PI, markPos, refMark);
        CROSS("module:LineLocator:lines",
              bestLineClosestPoint.x,
              bestLineClosestPoint.y,
              50,
              25,
              Drawings::ps_solid,
              ColorRGBA(0, 255, 0));

        drawPose(pose, ColorRGBA(0, 0, 255)); // Blue
        hypotheses.push_back(Hypothesis(pose, (markPos + bestLineClosestPoint).abs() * 0.5f, ColorRGBA(0, 0, 255)));

        // No mirror pose for the keeper! (assumed to always be on our side)
        if (!isKeeper) {
          drawPose(mirrorPose(pose), ColorRGBA(0, 0, 255));
          hypotheses.push_back(
            Hypothesis(mirrorPose(pose), (markPos + bestLineClosestPoint).abs() * 0.5f, ColorRGBA(0, 0, 255)));
        }
      }
    }
  }

  // Penalty shootouts only: Ball + closest penalty area line
  {
    if (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT && (ballSeenLower || ballSeenUpper) && lines.size() > 0 &&
        theOwnTeamInfo.teamNumber == theGameInfo.kickingTeam) {

      // Prioritize lower camera ball
      const Vector2<> ballPos = (ballSeenLower) ? ballPosLower : ballPosUpper;

      // Find penalty area line
      Line bestLine;
      float bestLineDist = 9999999999999.f; // HACK Big number
      for (size_t i = 0; i < lines.size(); ++i) {
        const Line& line = lines[i];

        const float len = (line.first - line.last).abs();
        const float refLen = 2.f * dim.yPosLeftPenaltyArea; // Near side of penalty area

        const float lineBallDist = fabsf(line.calculateDistToLine(ballPos));
        const float refLineBallDist = dim.xPosOpponentPenaltyArea - dim.xPosOpponentPenaltyMark;

        if (lineBallDist > 0.9 * refLineBallDist && lineBallDist < 1.1 * refLineBallDist && len > 0.5f * refLen &&
            len < 1.1f * refLen) {
          if (fabsf(line.d) < bestLineDist) {
            bestLine = line;
            bestLineDist = fabsf(line.d);
          }
        }
      }
      if (bestLineDist < 1.2f * (dim.xPosOpponentPenaltyArea - (dim.xPosOpponentPenaltyMark - 1000.f))) {
        const Vector2<> penaltyMark = Vector2<>(dim.xPosOpponentPenaltyMark, 0.f);
        pose = poseFromReference((bestLine.last - bestLine.first).angle(), -(float)M_PI / 2, ballPos, penaltyMark);

        // No mirror pose! We know we're on the opponent side of the field.
        drawPose(pose, ColorRGBA(255, 0, 0)); // Red
        hypotheses.push_back(Hypothesis(pose, (bestLine.first + bestLine.last).abs() * 0.5f, ColorRGBA(255, 0, 0)));
      }
    }
  }

  /* Find & provide hypotheses with lowest distance/uncertainty */

  auto& bestHypotheses = lineLocalization.hypotheses;
  {
    float lowestDist = INFINITY;
    for (const auto& hypothesis : hypotheses) {
      const float dist = hypothesis.dist;

      // HACK? Ignore all hypotheses on wrong side (i.e. own side) if in penalty shootout
      if (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT && theOwnTeamInfo.teamColor == theGameInfo.kickingTeam &&
          hypothesis.pose.translation.x < 0) {
        continue;
      }

      // If in initial state, ignore all hypotheses on wrong side (i.e. opponent side)
      if (theGameInfo.state == STATE_INITIAL && hypothesis.pose.translation.x > 0) {
        continue;
      }

      // If in set state, ignore all hypotheses on wrong side (i.e. opponent side)
      if (theGameInfo.state == STATE_SET && hypothesis.pose.translation.x > 0) {
        continue;
      }

      // If penalized, ignore all hypotheses on wrong side (i.e. opponent side)
      if (theRobotInfo.penalty != PENALTY_NONE && hypothesis.pose.translation.x > 0) {
        continue;
      }

      if (dist < lowestDist) {
        lowestDist = dist;
        bestHypotheses.clear();
        if (!isOutOfField(hypothesis.pose, outOfFieldMargin)) {
          bestHypotheses.push_back(hypothesis);
        }
        ASSERT(lowestDist > 0.f);
      } else if (fabsf(dist - lowestDist) < 1) {
        if (!isOutOfField(hypothesis.pose, outOfFieldMargin)) {
          bestHypotheses.push_back(hypothesis);
        }
      }
    }

    // Sanity check on the hypotheses
    if (lowestDist > maxDist) {
      bestHypotheses.clear();
    }

    // Draw thin circles around each best hypothesis
    for (size_t i = 0; i < bestHypotheses.size(); ++i) {
      const Hypothesis& hypothesis = bestHypotheses[i];
      const Pose2D& pose = hypothesis.pose;
      CIRCLE("module:LineLocator:poses",
             pose.translation.x,
             pose.translation.y,
             250,
             10,
             Drawings::ps_solid,
             ColorRGBA(hypothesis.color),
             Drawings::bs_null,
             ColorClasses::none);
    }
  }
}

MAKE_MODULE(LineLocator, Modeling)
