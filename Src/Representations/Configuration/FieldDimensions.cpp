/**
 * @file FieldDimensions.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Max Risler.
 */

#include "FieldDimensions.h"
#include "Core/Debugging/Modify.h"
#include "Core/Streams/InStreams.h"
#include "Core/System/BHAssert.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"
#include "Core/Communication/RoboCupControlData.h"
#include "Core/Debugging/DebugDrawings.h"
#include <limits>

using namespace std;

/**
 * Helper class that supports the use of symbolic values in float fields.
 */
class InSymbolicMapFile : public InMapFile {
private:
  std::unordered_map<std::string, float> values; /**< All symbolic values known. */
  const char* entry;                             /**< The name of the current entry processed. */

protected:
  /**
   * When reading a float, read a string instead. Try to replace symbolic value.
   * Symbolic value can be preceeded by a minus sign (without whitespace in between).
   */
  virtual void inFloat(float& value) {
    std::string buf;
    inString(buf);
    float sign = 1.f;
    if (buf[0] == '-') {
      sign = -1.f;
      buf = buf.substr(1);
    }

    std::unordered_map<std::string, float>::const_iterator i = values.find(buf);
    if (i != values.end()) {
      value = i->second * sign;
    } else if (!buf.empty() && (isdigit(buf[0]) || buf[0] == '.')) {
      value = (float)strtod(buf.c_str(), 0) * sign;
    } else {
      OUTPUT_ERROR("fieldDimensions.cfg: Unknown symbol '" << buf << "'");
    }

    if (entry) {
      values[entry] = value;
    }
  }

public:
  InSymbolicMapFile(const std::string& name) : InMapFile(name), entry(0) {}

  virtual void select(const char* name, int type, const char* (*enumToString)(unsigned char)) {
    Streaming::trimName(name);
    InMapFile::select(name, type, enumToString);
    entry = name;
  }
};

void FieldDimensions::load() {
  InSymbolicMapFile stream("fieldDimensions.cfg");
  ASSERT(stream.exists());
  stream >> *this;
}

Pose2D FieldDimensions::randomPoseOnField() const {
  Pose2D pose;
  Range<> xRange(xPosOpponentFieldBorder, xPosOwnFieldBorder);
  Range<> yRange(yPosLeftFieldBorder, yPosRightFieldBorder);
  do {

    pose = Pose2D::random(xRange, yRange, Range<>(-pi, pi));
  } while (!isInsideField(pose.translation));
  return pose;
}

Pose2D FieldDimensions::randomPoseOnCarpet() const {
  Pose2D pose;
  Range<> xRange(xPosOpponentFieldBorder, xPosOwnFieldBorder);
  Range<> yRange(yPosLeftFieldBorder, yPosRightFieldBorder);
  do {
    pose = Pose2D::random(xRange, yRange, Range<>(-pi, pi));
  } while (!isInsideCarpet(pose.translation));
  return pose;
}

void FieldDimensions::draw() const {
  drawLines();
  // drawCorners();
}

void FieldDimensions::drawLines() const {
  DECLARE_DEBUG_DRAWING("field lines", "drawingOnField");
  COMPLEX_DRAWING("field lines", {
    auto carpetBorderLines = getCarpetBorderLines();
    ASSERT(carpetBorderLines.size() <= 4);
    Vector2<> points[4];
    for (unsigned i = 0; i < carpetBorderLines.size(); ++i) {
      points[i] = carpetBorderLines[i].from;
    }
    POLYGON("field lines",
            (int)carpetBorderLines.size(),
            points,
            0,
            Drawings::ps_solid,
            ColorRGBA(0, 180, 0),
            Drawings::bs_solid,
            ColorRGBA(0, 140, 0));

    ColorRGBA lineColor(192, 192, 192);
    for (auto const& line : getAllFieldLines()) {
      Vector2<> source = line.from;
      Vector2<> target = line.to;
      LINE("field lines", source.x, source.y, target.x, target.y, fieldLinesWidth, Drawings::ps_solid, lineColor);
    }
  });
}

void FieldDimensions::drawPolygons(uint32_t ownColor) const {
  DECLARE_DEBUG_DRAWING("field polygons", "drawingOnField");
  COMPLEX_DRAWING("field polygons", {
    ColorRGBA own = ownColor == TEAM_BLUE ? ColorRGBA(50, 120, 127) : ColorRGBA(127, 50, 50);
    ColorRGBA opp = ownColor != TEAM_BLUE ? ColorRGBA(50, 120, 127) : ColorRGBA(127, 50, 50);

    Vector2<> goal[4];
    goal[0] = Vector2<>(xPosOwnGroundline - fieldLinesWidth * 0.5f, yPosLeftGoal);
    goal[1] = Vector2<>(xPosOwnGroundline - fieldLinesWidth * 0.5f, yPosRightGoal);
    goal[2] = Vector2<>(xPosOwnGoal, yPosRightGoal);
    goal[3] = Vector2<>(xPosOwnGoal, yPosLeftGoal);
    POLYGON("field polygons", 4, goal, 0, Drawings::ps_solid, own, Drawings::bs_solid, own);

    goal[0] = Vector2<>(xPosOpponentGroundline + fieldLinesWidth * 0.5f, yPosLeftGoal);
    goal[1] = Vector2<>(xPosOpponentGroundline + fieldLinesWidth * 0.5f, yPosRightGoal);
    goal[2] = Vector2<>(xPosOpponentGoal, yPosRightGoal);
    goal[3] = Vector2<>(xPosOpponentGoal, yPosLeftGoal);
    POLYGON("field polygons", 4, goal, 0, Drawings::ps_solid, opp, Drawings::bs_solid, opp);

    CIRCLE("field polygons",
           xPosOpponentGoalPost,
           yPosLeftGoal,
           50,
           0,
           Drawings::ps_solid,
           ColorRGBA(ColorClasses::yellow),
           Drawings::bs_solid,
           ColorRGBA(ColorClasses::yellow));
    CIRCLE("field polygons",
           xPosOpponentGoalPost,
           yPosRightGoal,
           50,
           0,
           Drawings::ps_solid,
           ColorRGBA(ColorClasses::yellow),
           Drawings::bs_solid,
           ColorRGBA(ColorClasses::yellow));

    CIRCLE("field polygons",
           xPosOwnGoalPost,
           yPosLeftGoal,
           50,
           0,
           Drawings::ps_solid,
           ColorRGBA(ColorClasses::yellow),
           Drawings::bs_solid,
           ColorRGBA(ColorClasses::yellow));
    CIRCLE("field polygons",
           xPosOwnGoalPost,
           yPosRightGoal,
           50,
           0,
           Drawings::ps_solid,
           ColorRGBA(ColorClasses::yellow),
           Drawings::bs_solid,
           ColorRGBA(ColorClasses::yellow));
  });
}

// void FieldDimensions::drawCorners() const {
//   DECLARE_DEBUG_DRAWING("field corners", "drawingOnField");

//   // #ifndef RELEASE
//   //   CornerClass c = CornerClass::xCorner;
//   // #endif
//   //   MODIFY_ENUM("fieldDimensions:cornerClass", c);
//   // COMPLEX_DRAWING("field corners", {
//   //   for (CornersTable::const_iterator i = corners[c].begin(); i != corners[c].end(); ++i) {
//   //     LARGE_DOT("field corners", i->x, i->y, ColorRGBA(255, 255, 255), ColorRGBA(255, 255, 255));
//   //   }
//   // });

//   // FIXME(albanesg): after replacing the bhuman enum with a proper C++11 enum class
//   // the code above that allows to select which corners should be drawn doesn't work
//   // so we just draw the x corners for now
//   COMPLEX_DRAWING("field corners", {
//     for (auto const& corner : this->xCorner) {
//       LARGE_DOT("field corners", corner.x, corner.y, ColorRGBA(255, 255, 255), ColorRGBA(255, 255, 255));
//     }
//   });
// }

void LinesTable::push(const Vector2<>& s, const Vector2<>& e, bool isPartOfCircle) {
  FieldDescriptionLine line;
  line.from = s;
  line.to = e;
  lines.push_back(line);
}

void LinesTable::pushCircle(const Vector2<>& center, float radius, int numOfSegments) {
  Vector2<> p1, p2;
  for (float a = 0; a <= pi_4; a += pi2 / numOfSegments) {
    p1 = Vector2<>(sin(a), cos(a)) * radius;
    if (a > 0) {
      push(center + p1, center + p2, true);
      push(center + Vector2<>(p1.x, -p1.y), center + Vector2<>(p2.x, -p2.y), true);
      push(center + Vector2<>(-p1.x, p1.y), center + Vector2<>(-p2.x, p2.y), true);
      push(center - p1, center - p2, true);
      push(center + Vector2<>(p1.y, p1.x), center + Vector2<>(p2.y, p2.x), true);
      push(center + Vector2<>(p1.y, -p1.x), center + Vector2<>(p2.y, -p2.x), true);
      push(center + Vector2<>(-p1.y, p1.x), center + Vector2<>(-p2.y, p2.x), true);
      push(center + Vector2<>(-p1.y, -p1.x), center + Vector2<>(-p2.y, -p2.x), true);
    }
    p2 = p1;
  }
}

bool LinesTable::isInside(const Vector2<>& v) const {
  // note:
  // This function assumes that the point (0,0) is inside and
  // that for any point inside the area the line to (0,0) belongs to the area too.

  Geometry::Line testLine(v, -v);
  for (vector<FieldDescriptionLine>::const_iterator i = lines.begin(); i != lines.end(); ++i) {
    float factor;
    auto border = Geometry::Line::fromPoints(i->from, i->to);
    if (Geometry::getIntersectionOfRaysFactor(border, testLine, factor)) {
      return false;
    }
  }
  return true;
}

float LinesTable::clip(Vector2<>& v) const {
  if (isInside(v)) {
    return 0;
  } else {
    Vector2<> old = v, v2;
    float minDist = 100000;
    for (vector<FieldDescriptionLine>::const_iterator i = lines.begin(); i != lines.end(); ++i) {
      Vector2<> diff = (Pose2D(old) - i->corner()).translation;
      if (diff.x < 0) {
        v2 = i->from;

      } else if (diff.x > i->length()) {
        v2 = (i->corner() + Pose2D(Vector2<>(i->length(), 0))).translation;
      } else {
        v2 = (i->corner() + Pose2D(Vector2<>(diff.x, 0))).translation;
      }
      float dist = (old - v2).abs();
      if (minDist > dist) {
        minDist = dist;
        v = v2;
      }
    }
    return (v - old).abs();
  }
}

bool LinesTable::getClosestPoint(Vector2<>& vMin, const Pose2D& p, int numberOfRotations, float minLength) const {
  int trueNumberOfRotations = numberOfRotations;
  if (numberOfRotations == 2) {
    numberOfRotations = 4;
  }

  // target angle -> target index
  float r = p.rotation / pi2 * numberOfRotations + 0.5f;
  if (r < 0) {
    r += numberOfRotations;
  }
  int targetRot = int(r);
  ASSERT(targetRot >= 0 && targetRot < numberOfRotations);
  targetRot %= trueNumberOfRotations;
  Vector2<> v2;
  float minDist = 100000;
  for (vector<FieldDescriptionLine>::const_iterator i = lines.begin(); i != lines.end(); ++i) {
    if (i->length() >= minLength) {
      // angle -> index
      float r = (i->rotation() + pi_2) / pi2 * numberOfRotations + 0.5f;
      if (r < 0) {
        r += numberOfRotations;
      } else if (r >= numberOfRotations) {
        r -= numberOfRotations;
      }
      int rot = int(r);
      ASSERT(rot >= 0 && rot < numberOfRotations);
      rot %= trueNumberOfRotations;

      // index must be target index
      if (rot == targetRot) {
        Vector2<> diff = (p - i->corner()).translation;
        if (diff.x < 0) {
          v2 = i->from;
        } else if (diff.x > i->length()) {
          v2 = (i->corner() + Pose2D(Vector2<>(i->length(), 0))).translation;
        } else {
          v2 = (i->corner() + Pose2D(Vector2<>(diff.x, 0))).translation;
        }
        Vector2<> vDiff = v2 - p.translation;
        float dist = vDiff.abs();
        if (minDist > dist) {
          minDist = dist;
          vMin = v2;
        }
      }
    }
  }
  return (minDist < 100000);
}

float LinesTable::getDistance(const Pose2D& pose) const {
  float minDist = 100000;
  for (vector<FieldDescriptionLine>::const_iterator i = lines.begin(); i != lines.end(); ++i) {
    Vector2<> v1 = (i->corner() - pose).translation,
              v2 = (i->corner() + Pose2D(Vector2<>(i->length(), 0)) - pose).translation;
    if (v1.y < 0 && v2.y > 0) {
      float dist = v1.x + (v2.x - v1.x) * -v1.y / (v2.y - v1.y);
      if (dist >= 0 && dist < minDist) {
        minDist = dist;
      }
    }
  }
  return minDist == 100000 ? -1 : minDist;
}

const Vector2<>& CornersTable::getClosest(const Vector2<>& p) const {
  ASSERT(!empty());
  float maxDistance2 = numeric_limits<float>().max();
  const Vector2<>* closest = 0;
  for (const_iterator i = begin(); i != end(); ++i) {
    Vector2<> diff = p - *i;
    float distance2 = diff * diff;
    if (maxDistance2 > distance2) {
      maxDistance2 = distance2;
      closest = &*i;
    }
  }
  return *closest;
}

Vector2<int> CornersTable::getClosest(const Vector2<int>& p) const {
  return Vector2<int>(getClosest(Vector2<>(p)));
}

std::vector<FieldDescriptionLine> FieldDimensions::getAllFieldLines() const {
  std::vector<FieldDescriptionLine> lines = getFieldLinesWithoutCenterCircle();
  auto center_circle_line_segs = getCenterCircleLineSegments();
  lines.insert(lines.end(), center_circle_line_segs.begin(), center_circle_line_segs.end());
  return lines;
}

std::vector<FieldDescriptionLine> FieldDimensions::getFieldLinesWithoutCenterCircle() const {

  if (fieldLines.empty()) {
    fieldLines = getFieldBorderLines();

    // center line
    fieldLines.emplace_back(Vector2<>(xPosHalfWayLine, yPosLeftSideline), Vector2<>(xPosHalfWayLine, yPosRightSideline));

    // penalty areas
    fieldLines.emplace_back(Vector2<>(xPosOwnGroundline, yPosLeftPenaltyArea),
                            Vector2<>(xPosOwnPenaltyArea, yPosLeftPenaltyArea));

    fieldLines.emplace_back(Vector2<>(xPosOwnPenaltyArea, yPosLeftPenaltyArea),
                            Vector2<>(xPosOwnPenaltyArea, yPosRightPenaltyArea));

    fieldLines.emplace_back(Vector2<>(xPosOwnPenaltyArea, yPosRightPenaltyArea),
                            Vector2<>(xPosOwnGroundline, yPosRightPenaltyArea));

    fieldLines.emplace_back(Vector2<>(xPosOpponentGroundline, yPosLeftPenaltyArea),
                            Vector2<>(xPosOpponentPenaltyArea, yPosLeftPenaltyArea));

    fieldLines.emplace_back(Vector2<>(xPosOpponentPenaltyArea, yPosLeftPenaltyArea),
                            Vector2<>(xPosOpponentPenaltyArea, yPosRightPenaltyArea));

    fieldLines.emplace_back(Vector2<>(xPosOpponentPenaltyArea, yPosRightPenaltyArea),
                            Vector2<>(xPosOpponentGroundline, yPosRightPenaltyArea));

    // goal boxes

    fieldLines.emplace_back(Vector2<>(xPosOwnGroundline, yPosLeftGoalBox), Vector2<>(xPosOwnGoalBox, yPosLeftGoalBox));

    fieldLines.emplace_back(Vector2<>(xPosOwnGoalBox, yPosLeftGoalBox), Vector2<>(xPosOwnGoalBox, yPosRightGoalBox));

    fieldLines.emplace_back(Vector2<>(xPosOwnGoalBox, yPosRightGoalBox), Vector2<>(xPosOwnGroundline, yPosRightGoalBox));

    fieldLines.emplace_back(Vector2<>(xPosOpponentGroundline, yPosLeftGoalBox),
                            Vector2<>(xPosOpponentGoalBox, yPosLeftGoalBox));

    fieldLines.emplace_back(Vector2<>(xPosOpponentGoalBox, yPosLeftGoalBox),
                            Vector2<>(xPosOpponentGoalBox, yPosRightGoalBox));

    fieldLines.emplace_back(Vector2<>(xPosOpponentGoalBox, yPosRightGoalBox),
                            Vector2<>(xPosOpponentGroundline, yPosRightGoalBox));

    // penalty and center marks
    fieldLines.emplace_back(Vector2<>(xPosOpponentPenaltyMark - fieldLinesWidth, 0),
                            Vector2<>(xPosOpponentPenaltyMark + fieldLinesWidth, 0));
    fieldLines.emplace_back(Vector2<>(xPosOpponentPenaltyMark, -fieldLinesWidth),
                            Vector2<>(xPosOpponentPenaltyMark, fieldLinesWidth));

    fieldLines.emplace_back(Vector2<>(xPosOwnPenaltyMark + fieldLinesWidth, 0),
                            Vector2<>(xPosOwnPenaltyMark - fieldLinesWidth, 0));
    fieldLines.emplace_back(Vector2<>(xPosOwnPenaltyMark, -fieldLinesWidth), Vector2<>(xPosOwnPenaltyMark, fieldLinesWidth));

    fieldLines.emplace_back(Vector2<>(-fieldLinesWidth, 0), Vector2<>(fieldLinesWidth, 0));
  }

  return fieldLines;
}

std::vector<FieldDescriptionLine> FieldDimensions::getFieldBorderLines() const {

  if (fieldBorderLines.empty()) {
    fieldBorderLines.emplace_back(Vector2<>(xPosOpponentGroundline, yPosRightSideline),
                                  Vector2<>(xPosOpponentGroundline, yPosLeftSideline));

    fieldBorderLines.emplace_back(Vector2<>(xPosOpponentGroundline, yPosLeftSideline),
                                  Vector2<>(xPosOwnGroundline, yPosLeftSideline));

    fieldBorderLines.emplace_back(Vector2<>(xPosOwnGroundline, yPosLeftSideline),
                                  Vector2<>(xPosOwnGroundline, yPosRightSideline));

    fieldBorderLines.emplace_back(Vector2<>(xPosOwnGroundline, yPosRightSideline),
                                  Vector2<>(xPosOpponentGroundline, yPosRightSideline));
  }

  return fieldBorderLines;
}

std::vector<FieldDescriptionLine> FieldDimensions::getCarpetBorderLines() const {
  if (carpetBorderLines.empty()) {
    carpetBorderLines.emplace_back(Vector2<>(xPosOpponentFieldBorder, yPosRightFieldBorder),
                                   Vector2<>(xPosOpponentFieldBorder, yPosLeftFieldBorder));
    carpetBorderLines.emplace_back(Vector2<>(xPosOpponentFieldBorder, yPosLeftFieldBorder),
                                   Vector2<>(xPosOwnFieldBorder, yPosLeftFieldBorder));
    carpetBorderLines.emplace_back(Vector2<>(xPosOwnFieldBorder, yPosLeftFieldBorder),
                                   Vector2<>(xPosOwnFieldBorder, yPosRightFieldBorder));
    carpetBorderLines.emplace_back(Vector2<>(xPosOwnFieldBorder, yPosRightFieldBorder),
                                   Vector2<>(xPosOpponentFieldBorder, yPosRightFieldBorder));
  }

  return carpetBorderLines;
}

bool FieldDimensions::isInsideCarpet(const Vector2<>& p) const {
  return isInside(getCarpetBorderLines(), p);
}

float FieldDimensions::clipToCarpet(Vector2<>& v) const {
  return clip(getCarpetBorderLines(), v);
}

bool FieldDimensions::isInsideField(const Vector2<>& p) const {
  return isInside(getFieldBorderLines(), p);
}

float FieldDimensions::clipToField(Vector2<>& v) const {
  return clip(getFieldBorderLines(), v);
}

FieldDescriptionLine::FieldDescriptionLine(Vector2<> from, Vector2<> to, bool isPartOfCircle) : FieldDescriptionLine() {
  this->from = from;
  this->to = to;
  this->isPartOfCircle = isPartOfCircle;
}

Pose2D FieldDescriptionLine::corner() const {
  return Pose2D(rotation(), from);
}

float FieldDescriptionLine::length() const {
  return (to - from).abs();
}

float FieldDescriptionLine::rotation() const {
  return (to - from).angle();
}

void FieldDescriptionLine::corner(const Pose2D& corner) {
  to = corner * Vector2<>(length(), 0.f);
  from = corner.translation;
}

bool FieldDimensions::isInside(const std::vector<FieldDescriptionLine>& lines, const Vector2<>& v) {
  Geometry::Line testLine(v, -v);
  for (auto const& line : lines) {
    float factor;
    auto border = Geometry::Line::fromPoints(line.from, line.to);
    if (Geometry::getIntersectionOfRaysFactor(border, testLine, factor)) {
      return false;
    }
  }
  return true;
}

float FieldDimensions::clip(const std::vector<FieldDescriptionLine>& lines, Vector2<>& v) {
  if (isInside(lines, v)) {
    return 0;
  } else {
    Vector2<> old = v, v2;
    float minDist = 100000;
    for (auto const& line : lines) {
      Vector2<> diff = (Pose2D(old) - line.corner()).translation;

      if (diff.x < 0) {
        v2 = line.from;

      } else if (diff.x > line.length()) {
        v2 = (line.corner() + Pose2D(Vector2<>(line.length(), 0))).translation;
      } else {
        v2 = (line.corner() + Pose2D(Vector2<>(diff.x, 0))).translation;
      }
      float dist = (old - v2).abs();
      if (minDist > dist) {
        minDist = dist;
        v = v2;
      }
    }
    return (v - old).abs();
  }
}

// std::vector<Vector2<>> FieldDimensions::getCornersOfClass(CornerClass cornerClass) const {
//   switch (cornerClass) {
//   case CornerClass::xCorner:
//     return this->xCorner;
//   case CornerClass::tCorner0:
//     return this->tCorner0;
//   case CornerClass::tCorner90:
//     return this->tCorner90;
//   case CornerClass::tCorner180:
//     return this->tCorner180;
//   case CornerClass::tCorner270:
//     return this->tCorner270;
//   case CornerClass::lCorner0:
//     return this->lCorner0;
//   case CornerClass::lCorner90:
//     return this->lCorner90;
//   case CornerClass::lCorner180:
//     return this->lCorner180;
//   case CornerClass::lCorner270:
//     return this->lCorner270;
//   default:
//     ASSERT(false);
//     return this->xCorner;
//   }
// }

std::vector<FieldDescriptionLine> FieldDimensions::getCenterCircleLineSegments() const {
  std::vector<FieldDescriptionLine> result;
  Vector2<> p1, p2;
  auto const& center = centerCircle.center;
  for (float a = 0; a <= pi_4; a += pi2 / centerCircle.numOfSegments) {
    p1 = Vector2<>(sin(a), cos(a)) * centerCircle.radius;
    if (a > 0) {
      result.emplace_back(center + p1, center + p2, true);
      result.emplace_back(center + Vector2<>(p1.x, -p1.y), center + Vector2<>(p2.x, -p2.y), true);
      result.emplace_back(center + Vector2<>(-p1.x, p1.y), center + Vector2<>(-p2.x, p2.y), true);
      result.emplace_back(center - p1, center - p2, true);
      result.emplace_back(center + Vector2<>(p1.y, p1.x), center + Vector2<>(p2.y, p2.x), true);
      result.emplace_back(center + Vector2<>(p1.y, -p1.x), center + Vector2<>(p2.y, -p2.x), true);
      result.emplace_back(center + Vector2<>(-p1.y, p1.x), center + Vector2<>(-p2.y, p2.x), true);
      result.emplace_back(center + Vector2<>(-p1.y, -p1.x), center + Vector2<>(-p2.y, -p2.x), true);
    }
    p2 = p1;
  }
  return result;
}