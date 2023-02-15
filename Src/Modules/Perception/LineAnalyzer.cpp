/**
 * @file LineAnalyzer.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#include "LineAnalyzer.h"
#include "Core/Debugging/DebugDrawings.h"
#include "Core/Streams/InStreams.h"
#include "Tools/Geometry/Shapes.h"
#include "Tools/Geometry/Transformations.h"
#include "Core/Range.h"
#include <algorithm>
#include <iostream>

using namespace std;

LineAnalyzer::LineAnalyzer() {
  ParameterWrapper wrapper;
  InMapFile inConfig("lineAnalyzer.cfg");
  inConfig >> wrapper;
  parameters = wrapper.parameters;
  circleParams = wrapper.circleParams;
  nonLineParams = wrapper.nonLineParams;
  banSectorParams = wrapper.banSectorParams;
}

void LineAnalyzer::update(LineAnalysis& lineAnalysis) {
  DECLARE_DEBUG_DRAWING("module:LineAnalyzer:LineSegments", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LineAnalyzer:NonLineSegments", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LineAnalyzer:LineSegmentsImg", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LineAnalyzer:banSectors", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LineAnalyzer:banSectorsImg", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LineAnalyzer:Lines1", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LineAnalyzer:CircleSpots", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LineAnalyzer:CircleSpots2", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LineAnalyzer:CircleSpotsImg", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LineAnalyzer:Lines2", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LineAnalyzer:Lines3", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LineAnalyzer:Intersections", "drawingOnField");
  // Nao Masserfassungsgeraet
  DECLARE_DEBUG_DRAWING("module:LineAnalyzer:naoMeter", "drawingOnImage");
  COMPLEX_DRAWING("module:LineAnalyzer:naoMeter", {
    Vector2<int> ppf;
    Vector2<int> ppf2;
    Vector2<> pp2 = theImageCoordinateSystem.toCorrected(Vector2<int>(260, 100));
    Vector2<> pp1 = theImageCoordinateSystem.toCorrected(Vector2<int>(160, 100));
    MODIFY("pp1", pp1);
    MODIFY("pp2", pp2);
    Geometry::calculatePointOnField((int)pp1.x, (int)pp1.y, theCameraMatrix, theCameraInfo, ppf);
    Geometry::calculatePointOnField((int)pp2.x, (int)pp2.y, theCameraMatrix, theCameraInfo, ppf2);
    LINE("module:LineAnalyzer:naoMeter", pp1.x, pp1.y, pp2.x, pp2.y, 2, Drawings::ps_solid, ColorClasses::black);
    LINE("module:LineAnalyzer:naoMeter", pp1.x, pp1.y - 5, pp1.x, pp1.y + 5, 2, Drawings::ps_solid, ColorClasses::black);
    LINE("module:LineAnalyzer:naoMeter", pp2.x, pp2.y - 5, pp2.x, pp2.y + 5, 2, Drawings::ps_solid, ColorClasses::black);
    DRAWTEXT("module:LineAnalyzer:naoMeter",
             pp1.x - 70,
             pp1.y + 50,
             50,
             ColorClasses::black,
             "Nao sagt: " << (ppf - ppf2).abs() << "mm");
  });

  MODIFY("parameters:LineAnalyzer", parameters);
  MODIFY("parameters:LineAnalyzerCircle", circleParams);
  MODIFY("parameters:LineAnalyzerNonLine", nonLineParams);
  MODIFY("parameters:LineAnalyzerBanSector", banSectorParams);

  STOP_TIME_ON_REQUEST("clearPercept", {
    lineSegs.clear();
    singleSegs.clear();
    lines.clear();
    lineAnalysis.intersections.clear();
    lineAnalysis.circle.found = false;
  });
  banSectors.clear();

  // STOP_TIME_ON_REQUEST("createLineSegments" , createLineSegments(singleSegs););
  STOP_TIME_ON_REQUEST("importLineSegments", importLineSegments(theLinePercept.fieldLineSegments, singleSegs););

  lineAnalysis.rawSegs.resize(singleSegs.size());
  copy(singleSegs.begin(), singleSegs.end(), lineAnalysis.rawSegs.begin());

  STOP_TIME_ON_REQUEST("createLines", createLines(lines, singleSegs););
  STOP_TIME_ON_REQUEST("analyzeSingleSegments", analyzeSingleSegments(singleSegs, lineAnalysis.circle, lines););
  STOP_TIME_ON_REQUEST("keepRightAngledLines", keepRightAngledLines(lines, singleSegs););

  std::vector<LineAnalysis::Intersection> intersections = lineAnalysis.intersections;
  STOP_TIME_ON_REQUEST("analyzeLines", analyzeLines(lines, intersections, lineAnalysis.circle, singleSegs););
  lineAnalysis.intersections.clear();
  for (const auto& el : intersections) {
    lineAnalysis.intersections.push_back(el);
  }

  lineAnalysis.singleSegs.resize(singleSegs.size());
  copy(singleSegs.begin(), singleSegs.end(), lineAnalysis.singleSegs.begin());
  lineAnalysis.lines.resize(lines.size());
  copy(lines.begin(), lines.end(), lineAnalysis.lines.begin());

  STOP_TIME_ON_REQUEST("drawLineAnalysis", {
    lineAnalysis.drawOnField(theFieldDimensions, 0);
    lineAnalysis.drawOnImage(theCameraMatrix, theCameraInfo, theFieldDimensions, 0, theImageCoordinateSystem);
    lineAnalysis.drawIn3D(theFieldDimensions, 0);
  });
}

void LineAnalyzer::importLineSegments(const vector<ShapePercept::LineSegment>& inputs,
                                      list<LineAnalysis::LineSegment>& outputs) {
  // cout << "Importing segments [\n";
  for (auto input = inputs.begin(); input != inputs.end(); input++) {
    Vector2<int> p1 = input->p1;
    Vector2<int> p2 = input->p2;
    Vector2<> pf1;
    Vector2<> pf2;

    // These are errors
    if (!Geometry::calculatePointOnField(theImageCoordinateSystem.toCorrected(p1), theCameraMatrix, theCameraInfo, pf1)) {
      continue;
    }
    if (!Geometry::calculatePointOnField(theImageCoordinateSystem.toCorrected(p2), theCameraMatrix, theCameraInfo, pf2)) {
      continue;
    }

    LineAnalysis::LineSegment output;
    output.p1Img = p1;
    output.p2Img = p2;
    output.p1 = pf1;
    output.p2 = pf2;

    Vector2<> diff = pf2 - pf1;
    output.alpha = diff.angle() + pi_2; // make alpha perpendicular to line segment
    // normalize alpha
    while (output.alpha < 0) {
      output.alpha += pi;
    }
    while (output.alpha >= pi) {
      output.alpha -= pi;
    }
    const float d1 = output.p1.x * cos(output.alpha) + output.p1.y * sin(output.alpha),
                d2 = output.p2.x * cos(output.alpha) + output.p2.y * sin(output.alpha);
    output.d = (d1 + d2) / 2.0f;

    outputs.push_back(output);

    // cout << "  {alpha: " << output.alpha << ", d: " << output.d << ", image: [" << output.p1Img.x << ", " <<
    // output.p1Img.y
    //   << "] -> [" << output.p2Img.x << ", " << output.p2Img.y << "], field: [" << output.p1.x << ", " << output.p1.y << "]
    //   -> ["
    //   << output.p2.x << ", " << output.p2.y << "]},\n";
  }
  // cout << "]\n";
}

void LineAnalyzer::createLines(list<LineAnalysis::ObservedFieldLine>& lines, list<LineAnalysis::LineSegment>& singleSegs) {
  // Hough Transformation fuer (ganz) arme....
  while (lineSegs.size() > 0) {
    // pick a segment...
    LineAnalysis::LineSegment seg = *lineSegs.begin();
    lineSegs.erase(lineSegs.begin());

    ARROW("module:LineAnalyzer:Lines1", seg.p1.x, seg.p1.y, seg.p2.x, seg.p2.y, 15, Drawings::ps_solid, ColorClasses::white);

    // collect supporters...
    vector<list<LineAnalysis::LineSegment>::iterator> supporters;
    float maxSegmentLength = 0;
    for (list<LineAnalysis::LineSegment>::iterator other = lineSegs.begin(); other != lineSegs.end(); other++) {
      if ((abs(other->alpha - seg.alpha) < parameters.maxAlphaDiff && abs(other->d - seg.d) < parameters.maxDDiff)) {
        const float sqr_length = (other->p1 - other->p2).squareAbs();
        if (sqr_length > maxSegmentLength) {
          maxSegmentLength = sqr_length;
        }
        supporters.push_back(other);
      } else if ((abs(abs(other->alpha - seg.alpha) - pi) < parameters.maxAlphaDiff &&
                  abs(other->d + seg.d) < parameters.maxDDiff)) {
        const float sqr_length = (other->p1 - other->p2).squareAbs();
        if (sqr_length > maxSegmentLength) {
          maxSegmentLength = sqr_length;
        }
        // make supporters all look into the same direction (alpha in [0...pi])
        if (other->alpha > seg.alpha) {
          other->alpha -= pi;
        } else {
          other->alpha += pi;
        }
        other->d *= -1;
        supporters.push_back(other);
      }
    }
    maxSegmentLength = std::sqrt(maxSegmentLength);

    // if you have enough supporters, you become a line
    if ((int)supporters.size() >= parameters.minSupporters && maxSegmentLength > parameters.minLineStartLength) {
      COMPLEX_DRAWING("module:LineAnalyzer:Lines1", {
        CROSS("module:LineAnalyzer:Lines1",
              (seg.p1.x + seg.p2.x) / 2,
              (seg.p1.y + seg.p2.y) / 2,
              20,
              20,
              Drawings::ps_solid,
              ColorClasses::red);
        DRAWTEXT(
          "module:LineAnalyzer:Lines1", seg.p1.x + 50, seg.p1.y + 100, 10, ColorClasses::black, (int)supporters.size());
      });
      LineAnalysis::ObservedFieldLine l;
      float d = seg.d, alpha = seg.alpha;
      l.dead = false;
      l.midLine = false;
      l.segments.push_back(seg);
      for (vector<list<LineAnalysis::LineSegment>::iterator>::const_iterator sup = supporters.begin();
           sup != supporters.end();
           sup++) {
        ARROW("module:LineAnalyzer:Lines1",
              (*sup)->p1.x,
              (*sup)->p1.y,
              (*sup)->p2.x,
              (*sup)->p2.y,
              15,
              Drawings::ps_solid,
              ColorClasses::red);
        ARROW("module:LineAnalyzer:Lines1",
              seg.p1.x,
              seg.p1.y,
              (*sup)->p1.x,
              (*sup)->p1.y,
              5,
              Drawings::ps_solid,
              ColorClasses::blue);
        d += (*sup)->d;
        alpha += (*sup)->alpha;
        l.segments.push_back(*(*sup));
      }
      for (vector<list<LineAnalysis::LineSegment>::iterator>::const_reverse_iterator sup = supporters.rbegin();
           sup != supporters.rend();
           sup++) {
        lineSegs.erase(*sup);
      }
      l.d = d / ((int)supporters.size() + 1);
      l.alpha = alpha / ((int)supporters.size() + 1);
      lines.push_back(l);
    } else {
      singleSegs.push_back(seg);
    }
  }
}

static inline float rightAngleDiff(float angle1, float angle2) {
  const float diff1 = fabs(wrapAngle(angle1 - angle2 - (float)M_PI));
  const float diff2 = fabs(wrapAngle(angle1 - angle2 - (float)M_PI / 2));
  const float diff3 = fabs(wrapAngle(angle1 - angle2));
  const float diff4 = fabs(wrapAngle(angle1 - angle2 + (float)M_PI / 2));
  return std::min<float>(diff1, std::min<float>(diff2, std::min<float>(diff3, diff4)));
}

static inline float segmentLengthSum(const vector<LineAnalysis::LineSegment>& segments) {
  float length = 0.f;
  for (auto segment = segments.begin(); segment != segments.end(); ++segment) {
    length += (segment->p1 - segment->p2).abs();
  }
  return length;
}

void LineAnalyzer::keepRightAngledLines(list<LineAnalysis::ObservedFieldLine>& lines,
                                        list<LineAnalysis::LineSegment>& singleSegs) {
  if (!parameters.rightAnglesOnly) {
    return;
  }
  if (lines.size() == 0) {
    return;
  }

  const float angleInc = (float)M_PI / 4 / 9;                            // every bucket is 5 degrees
  const float angleTol = parameters.rightAngleTol / 180.f * (float)M_PI; // allow lines up to +/- rightAngleTol degrees

  std::vector<float> angles(9, 0.f); // 9 buckets of 5 degrees each

  // Have each line cast votes for their angle
  for (auto line = lines.begin(); line != lines.end(); ++line) {
    const float angle = rightAngleDiff(line->alpha, 0.f);
    const size_t idx = (int)(angle / angleInc);

    float length = segmentLengthSum(line->segments);
    angles[idx] += length;
  }

  auto bestBucket = std::max_element(angles.begin(), angles.end());
  size_t bestIdx = std::distance(angles.begin(), bestBucket);

  // Calculate bestAngle as weighted average of lines in bestBucket and its neighbors
  float bestAngle = 0.f;
  float totalLength = 0.f;
  for (auto line = lines.begin(); line != lines.end(); ++line) {
    const float angle = rightAngleDiff(line->alpha, 0.f);
    const size_t idx = (int)(angle / angleInc);

    if (idx == bestIdx /* || idx == (bestIdx-1+9)%9 || idx == (bestIdx+1)%9*/) {
      float length = segmentLengthSum(line->segments);
      bestAngle += length * angle;
      totalLength += length;
    }
  }
  bestAngle /= totalLength;

  // OUTPUT(idText, text, "bestAngle = " << bestAngle);

  for (auto line = lines.begin(); line != lines.end(); /* increment depends on conditions below */) {
    const float angle = line->alpha;
    const float angleDiff = rightAngleDiff(angle, bestAngle);

    if (angleDiff <= angleTol) {
      // Keep this line
      ++line;
    } else {
      // Line was rejected: restore line segments
      for (size_t i = 0; i < line->segments.size(); ++i) {
        singleSegs.push_back(line->segments[i]);
      }
      line = lines.erase(line);
    }
  }
}

void LineAnalyzer::getFirstAndLastOfLine(LineAnalysis::ObservedFieldLine& line,
                                         Vector2<>& first,
                                         Vector2<>& last,
                                         bool updateLine) {
  Vector2<> p_ref = line.segments.at(0).p1;
  first = p_ref;
  last = p_ref;
  const Vector2<int>*firstImg = &line.segments.at(0).p1Img, *lastImg = firstImg;
  float first_dist = 0, last_dist = 0;

  Vector2<> mean;

  for (vector<LineAnalysis::LineSegment>::const_iterator seg = line.segments.begin(); seg != line.segments.end(); seg++) {
    mean.x += seg->p1.x + seg->p2.x;
    mean.y += seg->p1.y + seg->p2.y;

    const Vector2<> diffp1 = seg->p1 - p_ref, diffp2 = seg->p2 - p_ref;

    // if dist(p1, p_ref) > dist(first, p_ref) and dist(first, p1) < dist(p1, p_ref
    //-->means if p1 is farer away from p_ref as first and is on the same side of p_ref as first
    if (diffp1.squareAbs() > first_dist && (seg->p1 - first).squareAbs() <= diffp1.squareAbs()) {
      first = seg->p1;
      firstImg = &seg->p1Img;
      first_dist = (float)diffp1.squareAbs();
    } else if (diffp1.squareAbs() > last_dist && (seg->p1 - first).squareAbs() > (p_ref - first).squareAbs() &&
               (seg->p1 - last).squareAbs() <= diffp1.squareAbs()) {
      last = seg->p1;
      lastImg = &seg->p1Img;
      last_dist = (float)diffp1.squareAbs();
    }
    if (diffp2.squareAbs() > first_dist && (seg->p2 - first).squareAbs() <= diffp2.squareAbs()) {
      first = seg->p2;
      firstImg = &seg->p2Img;
      first_dist = (float)diffp2.squareAbs();
    } else if (diffp2.squareAbs() > last_dist && (seg->p2 - first).squareAbs() > (p_ref - first).squareAbs() &&
               (seg->p2 - last).squareAbs() <= diffp2.squareAbs()) {
      last = seg->p2;
      lastImg = &seg->p2Img;
      last_dist = (float)diffp2.squareAbs();
    }
  }

  if (updateLine) {
    ASSERT(line.segments.size());
    line.startInImage = Vector2<>(*firstImg);
    line.endInImage = Vector2<>(*lastImg);
    if ((int)line.segments.size() == 1) {
      line.alpha = line.segments.at(0).alpha;
      line.d = line.segments.at(0).d;
    } else {
      // improve alpha and d estimation by calculating the fitting line through all start/end points of the segments
      mean /= float((int)line.segments.size() * 2);
      CROSS("module:LineAnalyzer:Lines2", mean.x, mean.y, 80, 10, Drawings::ps_solid, ColorClasses::orange);
      float zaehlerSum = 0, nennerSum = 0;

      Vector2<> p1, p2;
      if (fabs(line.alpha - pi_2) > pi_4) {
        for (vector<LineAnalysis::LineSegment>::const_iterator seg = line.segments.begin(); seg != line.segments.end();
             seg++) {
          CROSS("module:LineAnalyzer:Lines2", seg->p1.x, seg->p1.y, 30, 10, Drawings::ps_solid, ColorClasses::blue);
          CROSS("module:LineAnalyzer:Lines2", seg->p2.x, seg->p2.y, 30, 10, Drawings::ps_solid, ColorClasses::blue);
          zaehlerSum += (seg->p1.y - mean.y) * (seg->p1.x - mean.x);
          nennerSum += sqr((seg->p1.y - mean.y));
          zaehlerSum += (seg->p2.y - mean.y) * (seg->p2.x - mean.x);
          nennerSum += sqr((seg->p2.y - mean.y));
        }
        float b = zaehlerSum / nennerSum, a = mean.x - b * mean.y;

        ASSERT(a == a);
        ASSERT(b == b);

        p1.y = mean.y;
        p1.x = a + b * p1.y;
        p2.y = mean.y + 1000;
        p2.x = a + b * p2.y;
        ARROW("module:LineAnalyzer:Lines2", p1.x, p1.y, p2.x, p2.y, 5, Drawings::ps_solid, ColorClasses::blue);
      } else {
        for (vector<LineAnalysis::LineSegment>::const_iterator seg = line.segments.begin(); seg != line.segments.end();
             seg++) {
          CROSS("module:LineAnalyzer:Lines2", seg->p1.x, seg->p1.y, 30, 10, Drawings::ps_solid, ColorClasses::blue);
          CROSS("module:LineAnalyzer:Lines2", seg->p2.x, seg->p2.y, 30, 10, Drawings::ps_solid, ColorClasses::blue);
          zaehlerSum += (seg->p1.x - mean.x) * (seg->p1.y - mean.y);
          nennerSum += sqr((seg->p1.x - mean.x));
          zaehlerSum += (seg->p2.x - mean.x) * (seg->p2.y - mean.y);
          nennerSum += sqr((seg->p2.x - mean.x));
        }
        float b = zaehlerSum / nennerSum, a = mean.y - b * mean.x;

        ASSERT(a == a);
        ASSERT(b == b);

        p1.x = mean.x;
        p1.y = a + b * p1.x;
        p2.x = mean.x + 1000;
        p2.y = a + b * p2.x;

        ARROW("module:LineAnalyzer:Lines2", p1.x, p1.y, p2.x, p2.y, 5, Drawings::ps_solid, ColorClasses::yellow);
      }

      line.alpha = (p1 - p2).angle() + pi_2;
      while (line.alpha < 0) {
        line.alpha += pi;
      }
      while (line.alpha >= pi) {
        line.alpha -= pi;
      }

      const float c = cos(line.alpha), s = sin(line.alpha);

      line.d = p1.x * c + p1.y * s;
    }
  }

  first = line.calculateClosestPointOnLine(first);
  last = line.calculateClosestPointOnLine(last);
}

void LineAnalyzer::analyzeLines(list<LineAnalysis::ObservedFieldLine>& lines,
                                vector<LineAnalysis::Intersection>& intersections,
                                LineAnalysis::CircleSpot& circle,
                                list<LineAnalysis::LineSegment>& singleSegs) {
  // the points first and last are the two points on the line which
  // have the greatest distance to each other ("endpoints")
  for (list<LineAnalysis::ObservedFieldLine>::iterator line = lines.begin(); line != lines.end(); line++) {
    getFirstAndLastOfLine(*line, line->first, line->last);
  }

  // delete lines if their circleSpot is near the found circle
  if (circle.found) {
    vector<list<LineAnalysis::ObservedFieldLine>::iterator> toDelete;
    for (list<LineAnalysis::ObservedFieldLine>::iterator l1 = lines.begin(); l1 != lines.end(); l1++) {
      Vector2<> line_mid = (l1->first + l1->last) / 2;
      CROSS("module:LineAnalyzer:CircleSpots", line_mid.x, line_mid.y, 30, 30, Drawings::ps_solid, ColorClasses::green);
      Vector2<> bla(line_mid + (l1->first - l1->last).rotate(pi_2).normalize(theFieldDimensions.centerCircleRadius));
      CROSS("module:LineAnalyzer:CircleSpots", bla.x, bla.y, 30, 30, Drawings::ps_solid, ColorClasses::green);
      if ((bla - circle.pos).squareAbs() < sqr(parameters.maxLineCircleDist)) {
        toDelete.push_back(l1);
      } else {
        Vector2<> bla(line_mid + (l1->first - l1->last).rotate(-pi_2).normalize(theFieldDimensions.centerCircleRadius));
        CROSS("module:LineAnalyzer:CircleSpots", bla.x, bla.y, 30, 30, Drawings::ps_solid, ColorClasses::green);
        if ((bla - circle.pos).squareAbs() < sqr(parameters.maxLineCircleDist)) {
          toDelete.push_back(l1);
        }
      }
    }
    for (vector<list<LineAnalysis::ObservedFieldLine>::iterator>::iterator t = toDelete.begin(); t != toDelete.end(); t++) {
      lines.erase(*t);
    }
  }

  // delete lines if they have at least two segments which overlap (these might be robot legs)
  vector<list<LineAnalysis::ObservedFieldLine>::iterator> toDelete;
  for (list<LineAnalysis::ObservedFieldLine>::iterator line = lines.begin(); line != lines.end(); line++) {
    if (line->dead) {
      continue;
    }

    float alpha2 = line->alpha + pi_2;

    vector<LineAnalysis::LineSegment>::iterator other;
    for (vector<LineAnalysis::LineSegment>::iterator seg = line->segments.begin(); seg != line->segments.end(); seg++) {
      float d1 = seg->p1.x * cos(alpha2) + seg->p1.y * sin(alpha2), d2 = seg->p2.x * cos(alpha2) + seg->p2.y * sin(alpha2);
      Range<> clipper(min(d1, d2), max(d1, d2));

      other = seg;
      other++;
      for (; other != line->segments.end(); other++) {
        float otherd1 = clipper.limit(other->p1.x * cos(alpha2) + other->p1.y * sin(alpha2)),
              otherd2 = clipper.limit(other->p2.x * cos(alpha2) + other->p2.y * sin(alpha2));

        if (fabs(otherd1 - otherd2) > parameters.maxOverlapLength) {
          LINE("module:LineAnalyzer:Lines3",
               seg->p1.x,
               seg->p1.y,
               seg->p2.x,
               seg->p2.y,
               5,
               Drawings::ps_solid,
               ColorClasses::red);
          LINE("module:LineAnalyzer:Lines3",
               other->p1.x,
               other->p1.y,
               other->p2.x,
               other->p2.y,
               5,
               Drawings::ps_solid,
               ColorClasses::red);
          toDelete.push_back(line);
          goto breakOuter;
        }
      }
    }
  breakOuter:;
  }
  for (vector<list<LineAnalysis::ObservedFieldLine>::iterator>::iterator t = toDelete.begin(); t != toDelete.end(); t++) {
    lines.erase(*t);
  }

  // find lines which are allmost parallel
  toDelete.clear();
  for (list<LineAnalysis::ObservedFieldLine>::iterator line = lines.begin(); line != lines.end(); line++) {
    if (line->dead) {
      continue;
    }
    list<LineAnalysis::ObservedFieldLine>::iterator other = line;
    other++;
    for (; other != lines.end(); other++) {
      if (other->dead) {
        continue;
      }

      float alphaDiff = line->alpha - other->alpha;
      while (alphaDiff < -pi_2) {
        alphaDiff += pi;
      }
      while (alphaDiff >= pi_2) {
        alphaDiff -= pi;
      }
      // if endpoints of the other line are close to the line
      if (((abs(line->calculateDistToLine(other->first)) < parameters.maxLineUniteDist &&
            abs(line->calculateDistToLine(other->last)) < parameters.maxLineUniteDist) ||
           (abs(other->calculateDistToLine(line->first)) < parameters.maxLineUniteDist &&
            abs(other->calculateDistToLine(line->last)) < parameters.maxLineUniteDist)) &&
          abs(alphaDiff) < parameters.maxLineUniteAlphaDiff) {
        // line->segments.insert(line->segments.end(), other->segments.begin(), other->segments.end());
        for (auto& segment : other->segments) {
          line->segments.push_back(segment);
        }
        line->d = (line->d + other->d) / 2;
        if (line->alpha - other->alpha > pi_2) {
          other->alpha += pi;
        } else if (other->alpha - line->alpha > pi_2) {
          other->alpha -= pi;
        }
        line->alpha = (line->alpha + other->alpha) / 2;
        if (line->alpha < 0) {
          line->alpha += pi;
        }
        getFirstAndLastOfLine(*line, (*line).first, (*line).last);
        toDelete.push_back(other);
        other->dead = true;
      }
    }
  }
  for (vector<list<LineAnalysis::ObservedFieldLine>::iterator>::iterator t = toDelete.begin(); t != toDelete.end(); t++) {
    lines.erase(*t);
  }

  // add singleSegments where the start and end pos is close to a line to the line
  vector<list<LineAnalysis::LineSegment>::iterator> ttoDelete;
  for (list<LineAnalysis::LineSegment>::iterator seg = singleSegs.begin(); seg != singleSegs.end(); seg++) {
    for (list<LineAnalysis::ObservedFieldLine>::iterator line = lines.begin(); line != lines.end(); line++) {

      if (abs(seg->p1.x * cos(line->alpha) + seg->p1.y * sin(line->alpha) - line->d) < parameters.maxLineSingleSegDist &&
          abs(seg->p2.x * cos(line->alpha) + seg->p2.y * sin(line->alpha) - line->d) < parameters.maxLineSingleSegDist) {
        float firstToLast = (float)(line->last - line->first).abs();
        const Vector2<> segMid = (seg->p1 + seg->p2) / 2.f;
        CROSS("module:LineAnalyzer:Lines2", segMid.x, segMid.y, 30, 20, Drawings::ps_solid, ColorClasses::yellow);
        if ((firstToLast < (line->last - segMid).abs() || firstToLast < (line->first - segMid).abs())) {
          // seg is not between first and last
          const float minToLine = (line->last - segMid).abs() > (line->first - segMid).abs() ? (line->first - segMid).abs()
                                                                                             : (line->last - segMid).abs();

          COMPLEX_DRAWING("module:LineAnalyzer:Lines2", {
            DRAWTEXT("module:LineAnalyzer:Lines2", segMid.x + 20, segMid.y, 10, ColorClasses::black, minToLine);
          });

          if (minToLine > parameters.maxLineSingleSegDist2) {
            continue;
          }
          CROSS("module:LineAnalyzer:Lines2", segMid.x, segMid.y, 30, 20, Drawings::ps_solid, ColorClasses::red);
        } else {
          CROSS("module:LineAnalyzer:Lines2", segMid.x, segMid.y, 30, 20, Drawings::ps_solid, ColorClasses::blue);
        }

        line->segments.push_back(*seg);
        ttoDelete.push_back(seg);
        getFirstAndLastOfLine(*line, line->first, line->last, false);
        break;
      }
    }
  }
  for (vector<list<LineAnalysis::LineSegment>::iterator>::iterator d = ttoDelete.begin(); d != ttoDelete.end(); d++) {
    singleSegs.erase(*d);
  }

  // delete lines which do not "hard cover" (length / sum(segemnts.length)) enough
  toDelete.clear();
  for (list<LineAnalysis::ObservedFieldLine>::iterator l1 = lines.begin(); l1 != lines.end(); l1++) {
    float hardcover = 0;
    for (vector<LineAnalysis::LineSegment>::iterator seg = l1->segments.begin(); seg != l1->segments.end(); seg++) {
      hardcover += (seg->p1 - seg->p2).abs();
    }
    const float hardcoverRatio = hardcover / (l1->first - l1->last).abs();
    if (hardcoverRatio < parameters.minHardcover) {
      toDelete.push_back(l1);
    }
  }
  for (vector<list<LineAnalysis::ObservedFieldLine>::iterator>::iterator t = toDelete.begin(); t != toDelete.end(); t++) {
    for (vector<LineAnalysis::LineSegment>::iterator seg = (*(*t)).segments.begin(); seg != (*(*t)).segments.end(); seg++) {
      singleSegs.push_back(*seg);
    }
    lines.erase(*t);
  }

  // find intersections
  for (list<LineAnalysis::ObservedFieldLine>::iterator l1 = lines.begin(); l1 != lines.end(); l1++) {
    list<LineAnalysis::ObservedFieldLine>::iterator l2 = l1;
    for (l2++; l2 != lines.end(); l2++) {
      float alphaDiff = l1->alpha - l2->alpha;
      while (alphaDiff < -pi_2) {
        alphaDiff += pi;
      }
      while (alphaDiff >= pi_2) {
        alphaDiff -= pi;
      }
      if (abs(alphaDiff) < parameters.minIntersectionAlphaDiff) {
        continue;
      }

      if ((l1->first - l1->last).squareAbs() < sqr(parameters.minIntersectionLength) &&
          (l2->first - l2->last).squareAbs() < sqr(parameters.minIntersectionLength)) {
        continue;
      }

      // zwei hessesche normaleformen gleichsetzen und aufloesen, dann kommt das bei raus
      const float zaehler = l1->d - (l2->d * cos(l1->alpha) / cos(l2->alpha)),
                  nenner = sin(l1->alpha) - (sin(l2->alpha) * cos(l1->alpha) / cos(l2->alpha));
      const float y_s = zaehler / nenner, x_s = (l1->d - y_s * sin(l1->alpha)) / cos(l1->alpha);

      if (y_s == y_s && x_s == x_s) // intersection exists -> not paralel || ident
      {
        const Vector2<> s_p(x_s, y_s);
        // this is some freay stuff which determines in which relation the
        // point s_p is to l1->first/last and l2->first/last given s_p is the
        // intersectionpoint of l1 and l2
        // distToLx = ( -min(dist(sp,last/first)) if in between,  )
        //           (  min(dist(s_p,last/first) else)           )
        float spToFirst = (float)(s_p - l1->first).abs(), spToLast = (float)(s_p - l1->last).abs(),
              firstToLast = (float)(l1->first - l1->last).abs();
        float distToL1 = 0, distToL2 = 0;
        if (spToFirst < firstToLast && spToLast < firstToLast) {
          // sp is between first and last
          distToL1 = -(spToFirst > spToLast ? spToLast : spToFirst);
        } else if (spToFirst >= firstToLast) {
          // sp is closer to last
          distToL1 = spToLast;
        } else if (spToLast >= firstToLast) {
          // sp is closer to first
          distToL1 = spToFirst;
        } else {
          ASSERT(false);
        }
        spToFirst = (float)(s_p - l2->first).abs(), spToLast = (float)(s_p - l2->last).abs(),
        firstToLast = (float)(l2->first - l2->last).abs();
        if (spToFirst < firstToLast && spToLast < firstToLast) {
          // sp is between first and last
          distToL2 = -(spToFirst > spToLast ? spToLast : spToFirst);
        } else if (spToFirst >= firstToLast) {
          // sp is closer to last
          distToL2 = spToLast;
        } else if (spToLast >= firstToLast) {
          // sp is closer to first
          distToL2 = spToFirst;
        } else {
          ASSERT(false);
        }
        // end freaky stuff

        LineAnalysis::Intersection inter;
        inter.pos = Vector2<>(x_s, y_s);
        Vector2<> t1 = l1->first - l1->last, t2 = l2->first - l2->last;
        // this checks whether the intersection point is closer to first
        // or to last and if it is closer to first we need to flip the
        // direction
        if ((l1->first - inter.pos).squareAbs() < (l1->last - inter.pos).squareAbs()) {
          t1 = l1->last - l1->first;
        }
        if ((l2->first - inter.pos).squareAbs() < (l2->last - inter.pos).squareAbs()) {
          t2 = l2->last - l2->first;
        }
        // this is the heading of the intersection (to l1 and l2)
        Vector2<> dirL1 = Vector2<>(t1).normalize(), dirL2 = Vector2<>(t2).normalize();

        if (distToL1 < -parameters.minTToEnd && distToL2 < -parameters.minTToEnd) {
          ARROW("module:LineAnalyzer:Intersections",
                x_s,
                y_s,
                l1->last.x,
                l1->last.y,
                5,
                Drawings::ps_solid,
                ColorClasses::yellow);
          ARROW("module:LineAnalyzer:Intersections",
                x_s,
                y_s,
                l2->last.x,
                l2->last.y,
                5,
                Drawings::ps_solid,
                ColorClasses::yellow);
          // this is a X
          inter.type = LineAnalysis::Intersection::X;
          inter.dir1 = dirL1;
          inter.dir2 = dirL2;
          inter.first1 = l1->first;
          inter.last1 = l1->last;
          inter.first2 = l2->first;
          inter.last2 = l2->last;
          intersections.push_back(inter);
        } else if ((distToL1 < -parameters.minTToEnd && distToL2 < parameters.maxTFromEnd) ||
                   (distToL2 < -parameters.minTToEnd && distToL1 < parameters.maxTFromEnd)) {
          ARROW("module:LineAnalyzer:Intersections",
                x_s,
                y_s,
                l1->last.x,
                l1->last.y,
                5,
                Drawings::ps_solid,
                ColorClasses::yellow);
          ARROW("module:LineAnalyzer:Intersections",
                x_s,
                y_s,
                l2->last.x,
                l2->last.y,
                5,
                Drawings::ps_solid,
                ColorClasses::yellow);
          // this is a T
          inter.type = LineAnalysis::Intersection::T;
          if (distToL2 < -parameters.minTToEnd && distToL1 < parameters.maxTFromEnd) {
            // l2 is the intersected line (the upper part of the T)
            inter.dir1 = dirL1;
            inter.dir2 = dirL2;
            inter.first1 = l1->first;
            inter.last1 = l1->last;
            inter.first2 = l2->first;
            inter.last2 = l2->last;
          } else {
            // l1 is the intersected line (the upper part of the T)
            inter.dir1 = dirL2;
            inter.dir2 = dirL1;
            inter.first1 = l2->first;
            inter.last1 = l2->last;
            inter.first2 = l1->first;
            inter.last2 = l1->last;
          }
          intersections.push_back(inter);
        } else if (distToL1 < parameters.maxTFromEnd && distToL2 < parameters.maxTFromEnd) {
          ARROW("module:LineAnalyzer:Intersections",
                x_s,
                y_s,
                l1->last.x,
                l1->last.y,
                5,
                Drawings::ps_solid,
                ColorClasses::yellow);
          ARROW("module:LineAnalyzer:Intersections",
                x_s,
                y_s,
                l2->last.x,
                l2->last.y,
                5,
                Drawings::ps_solid,
                ColorClasses::yellow);
          // this is a L
          inter.type = LineAnalysis::Intersection::L;
          inter.dir1 = dirL1;
          inter.dir2 = dirL2;
          inter.first1 = l1->first;
          inter.last1 = l1->last;
          inter.first2 = l2->first;
          inter.last2 = l2->last;
          intersections.push_back(inter);
        }
      }
    }
  }

  // find "mittellinie"
  if (circle.found) {
    list<LineAnalysis::ObservedFieldLine>::iterator closestLine;
    int minDist = -1;
    for (list<LineAnalysis::ObservedFieldLine>::iterator l1 = lines.begin(); l1 != lines.end(); l1++) {
      const int dist = (int)abs(l1->calculateDistToLine(circle.pos));
      if (dist < parameters.maxMidLineToCircleDist && (dist < minDist || minDist == -1) &&
          (l1->first - l1->last).squareAbs() > sqr(parameters.minMidLineLength)) {
        closestLine = l1;
        minDist = dist;
      }
    }

    if (minDist != -1) {
      closestLine->midLine = true;
      circle.pos = closestLine->calculateClosestPointOnLine(circle.pos);

      // intersections
      const Vector2<> midLineDir = (closestLine->first - closestLine->last).normalize(theFieldDimensions.centerCircleRadius);
      LineAnalysis::Intersection inter;

      inter.pos = circle.pos + midLineDir;
      inter.dir1 = Vector2<>(midLineDir).normalize();
      inter.dir2 = inter.dir1;
      inter.dir2.rotateLeft();
      inter.first1 = closestLine->first;
      inter.last1 = closestLine->last;
      inter.first2 = Vector2<>();
      inter.last2 = Vector2<>();
      inter.type = LineAnalysis::Intersection::X;
      intersections.push_back(inter);

      inter.pos = circle.pos - midLineDir;
      inter.dir1 = Vector2<>(midLineDir).normalize();
      inter.dir2 = inter.dir1;
      inter.dir2.rotateLeft();
      inter.first1 = closestLine->first;
      inter.last1 = closestLine->last;
      inter.first2 = Vector2<>();
      inter.last2 = Vector2<>();
      inter.type = LineAnalysis::Intersection::X;
      intersections.push_back(inter);
    }
  }
}

void LineAnalyzer::analyzeSingleSegments(list<LineAnalysis::LineSegment>& singleSegs,
                                         LineAnalysis::CircleSpot& circle,
                                         list<LineAnalysis::ObservedFieldLine>& lines) {

  list<LineAnalysis::CircleSpot> circleSpots;
  list<LineAnalysis::CircleSpot> circleSpots2;
  LineAnalysis::CircleSpot spot;

  list<LineAnalysis::LineSegment>::iterator seg2;
  for (list<LineAnalysis::LineSegment>::iterator seg = singleSegs.begin(); seg != singleSegs.end(); seg++) {
    ARROW("module:LineAnalyzer:CircleSpots",
          seg->p1.x,
          seg->p1.y,
          seg->p2.x,
          seg->p2.y,
          20,
          Drawings::ps_solid,
          ColorClasses::blue);

    const Vector2<> seg_dir = seg->p1 - seg->p2;
    const Vector2<> seg_mid = (seg->p1 + seg->p2) / 2;
    const Vector2<> seg_norm = Vector2<>(seg_dir.x, seg_dir.y).rotateLeft();

    ASSERT(seg->p1 != seg->p2);
    const Vector2<> spot1 = seg_mid + (seg->p1 - seg->p2).rotate(pi_2).normalize(theFieldDimensions.centerCircleRadius);
    spot.pos = spot1;
    spot.iterator = seg;
    LINE("module:LineAnalyzer:CircleSpots2",
         spot.pos.x,
         spot.pos.y,
         seg_mid.x,
         seg_mid.y,
         5,
         Drawings::ps_solid,
         ColorClasses::yellow);
    CROSS("module:LineAnalyzer:CircleSpots2", spot.pos.x, spot.pos.y, 20, 20, Drawings::ps_solid, ColorClasses::yellow);
    circleSpots2.push_back(spot);
    const Vector2<> spot2 = seg_mid + (seg->p1 - seg->p2).rotate(-pi_2).normalize(theFieldDimensions.centerCircleRadius);
    spot.pos = spot2;
    spot.iterator = seg;
    LINE("module:LineAnalyzer:CircleSpots2",
         spot.pos.x,
         spot.pos.y,
         seg_mid.x,
         seg_mid.y,
         5,
         Drawings::ps_solid,
         ColorClasses::yellow);
    CROSS("module:LineAnalyzer:CircleSpots2", spot.pos.x, spot.pos.y, 20, 20, Drawings::ps_solid, ColorClasses::yellow);
    circleSpots2.push_back(spot);

    if (seg_dir.squareAbs() < sqr(circleParams.minSegmentLength) ||
        seg_dir.squareAbs() > sqr(2 * theFieldDimensions.centerCircleRadius) ||
        (seg->p1Img - seg->p2Img).squareAbs() < sqr(circleParams.minSegmentImgLength)) {
      continue;
    }

    LINE("module:LineAnalyzer:CircleSpots",
         seg_mid.x,
         seg_mid.y,
         seg_mid.x + seg_norm.x,
         seg_mid.y + seg_norm.y,
         20,
         Drawings::ps_solid,
         ColorClasses::orange);

    seg2 = seg;
    seg2++;
    for (; seg2 != singleSegs.end(); seg2++) {
      const Vector2<> seg2_dir = seg2->p1 - seg2->p2;
      if (seg2_dir.squareAbs() < sqr(circleParams.minSegmentLength) ||
          seg2_dir.squareAbs() > sqr(2 * theFieldDimensions.centerCircleRadius) ||
          (seg2->p1Img - seg2->p2Img).squareAbs() < sqr(circleParams.minSegmentImgLength)) {
        continue;
      }

      if ((seg->p1 - seg2->p1).squareAbs() < sqr(circleParams.maxNgbhDist) ||
          (seg->p1 - seg2->p2).squareAbs() < sqr(circleParams.maxNgbhDist) ||
          (seg->p2 - seg2->p1).squareAbs() < sqr(circleParams.maxNgbhDist) ||
          (seg->p2 - seg2->p2).squareAbs() < sqr(circleParams.maxNgbhDist)) {
        const Vector2<> seg2_mid = (seg2->p1 + seg2->p2) / 2;
        LINE("module:LineAnalyzer:CircleSpots",
             seg_mid.x,
             seg_mid.y,
             seg2_mid.x,
             seg2_mid.y,
             20,
             Drawings::ps_solid,
             ColorClasses::red);
        const Vector2<> seg2_norm = Vector2<>(seg2_dir.x, seg2_dir.y).rotateLeft();

        const Vector2<> p1 = seg_mid;
        const Vector2<> p2 = seg_mid + seg_norm;
        ;
        const Vector2<> p3 = seg2_mid;
        const Vector2<> p4 = seg2_mid + seg2_norm;

        const float zaehler1 = (p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x);
        const float nenner1 = (p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y);
        float X1factor = zaehler1 / (float)nenner1;

        const float zaehler2 = (p2.x - p1.x) * (p1.y - p3.y) - (p2.y - p1.y) * (p1.x - p3.x);
        const float nenner2 = (p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y);
        const float X2factor = zaehler2 / nenner2;

        const Vector2<> t = p2 - p1;
        const Vector2<> inter = p1 + Vector2<>(t.x * X1factor, t.y * X1factor);

        if (abs(abs(seg_norm.abs() * X1factor) - theFieldDimensions.centerCircleRadius) > circleParams.maxRadiusError ||
            abs(abs(seg2_norm.abs() * X2factor) - theFieldDimensions.centerCircleRadius) > circleParams.maxRadiusError) {
          continue;
        }

        const int X1Sign = X1factor > 0 ? 1 : -1;
        const int X2Sign = X2factor > 0 ? 1 : -1;
        const Vector2<> i1 = seg_mid + Vector2<>(seg_norm).normalize(theFieldDimensions.centerCircleRadius * X1Sign);
        const Vector2<> i2 = seg2_mid + Vector2<>(seg2_norm).normalize(theFieldDimensions.centerCircleRadius * X2Sign);

        // CROSS("module:LineAnalyzer:CircleSpots", inter.x, inter.y, 40, 20, Drawings::ps_solid, ColorClasses::red);
        CROSS("module:LineAnalyzer:CircleSpots", i1.x, i1.y, 40, 20, Drawings::ps_solid, ColorClasses::blue);
        CROSS("module:LineAnalyzer:CircleSpots", i2.x, i2.y, 40, 20, Drawings::ps_solid, ColorClasses::blue);
        LINE("module:LineAnalyzer:CircleSpots",
             seg_mid.x,
             seg_mid.y,
             inter.x,
             inter.y,
             10,
             Drawings::ps_solid,
             ColorClasses::orange);
        LINE("module:LineAnalyzer:CircleSpots",
             seg2_mid.x,
             seg2_mid.y,
             inter.x,
             inter.y,
             10,
             Drawings::ps_solid,
             ColorClasses::orange);
        spot.pos = i1;
        circleSpots.push_back(spot);
        spot.pos = i2;
        circleSpots.push_back(spot);
      }
    }
  }

  // Hough Transformation fuer (ganz) arme ;-)
  const int sqrMaxSupporterDist = sqr(circleParams.maxSupporterDist);
  list<list<LineAnalysis::LineSegment>::iterator> toDelete;
  for (list<LineAnalysis::CircleSpot>::iterator spot_iter = circleSpots.begin(); spot_iter != circleSpots.end();
       spot_iter++) {
    spot = *spot_iter;
    Vector2<> center(0, 0);

    vector<list<LineAnalysis::CircleSpot>::iterator> supporters;

    for (list<LineAnalysis::CircleSpot>::iterator other = circleSpots.begin(); other != circleSpots.end(); other++) {
      if ((other->pos - spot.pos).squareAbs() < sqrMaxSupporterDist) {
        supporters.push_back(other);
        center += other->pos;
      }
    }

    if ((int)supporters.size() >= circleParams.minSupporters) {
      center /= (float)supporters.size();

      // collect second round of supporters
      for (list<LineAnalysis::CircleSpot>::iterator other = circleSpots2.begin(); other != circleSpots2.end(); other++) {
        if ((other->pos - center).squareAbs() < sqr(circleParams.maxSupporterDist2)) {
          toDelete.push_back(other->iterator);
        }
      }

      circle.pos = center;
      circle.found = true;
      circle.lastSeen = theFrameInfo.time;
      CIRCLE("module:LineAnalyzer:CircleSpots",
             circle.pos.x,
             circle.pos.y,
             circleParams.maxSupporterDist,
             30,
             Drawings::ps_solid,
             ColorClasses::blue,
             Drawings::bs_null,
             ColorClasses::blue);
      CIRCLE("module:LineAnalyzer:CircleSpots",
             circle.pos.x,
             circle.pos.y,
             circleParams.maxSupporterDist2,
             30,
             Drawings::ps_solid,
             ColorClasses::blue,
             Drawings::bs_null,
             ColorClasses::blue);
      break;
    }
  }
  for (list<list<LineAnalysis::LineSegment>::iterator>::iterator d = toDelete.begin(); d != toDelete.end(); ++d) {
    list<list<LineAnalysis::LineSegment>::iterator>::iterator i = d;
    for (++i; i != toDelete.end();) {
      if (*i == *d) {
        i = toDelete.erase(i);
      } else {
        ++i;
      }
    }
    singleSegs.erase(*d);
  }

  // a single segment is assumed to be a line if it's size is sufficent (and it's not part of the circle)
  toDelete.clear();
  for (list<LineAnalysis::LineSegment>::iterator seg = singleSegs.begin(); seg != singleSegs.end(); seg++) {
    if ((seg->p1 - seg->p2).squareAbs() > sqr(parameters.minLineSingleRegionLength)) {
      LineAnalysis::ObservedFieldLine l;
      l.d = seg->d;
      l.alpha = seg->alpha;
      l.segments.push_back(*seg);
      l.dead = false;
      l.midLine = false;
      lines.push_back(l);
      toDelete.push_back(seg);
    }
  }
  for (list<list<LineAnalysis::LineSegment>::iterator>::iterator d = toDelete.begin(); d != toDelete.end(); ++d) {
    list<list<LineAnalysis::LineSegment>::iterator>::iterator i = d;
    for (++i; i != toDelete.end();) {
      if (*i == *d) {
        i = toDelete.erase(i);
      } else {
        ++i;
      }
    }
    singleSegs.erase(*d);
  }
}

MAKE_MODULE(LineAnalyzer, Perception)
