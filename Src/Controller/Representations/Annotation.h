/**
 * @file Annotation.h
 *
 * Declaration of class Annotation
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <QMetaType>
#include <boost/variant.hpp>
#include "Dataset.h"
#include "Core/Math/Vector2.h"
#include "Core/Streams/AutoStreamable.h"

class StreamableCircleAnnotation;
class StreamablePolygonAnnotation;

/**
 * @class Annotation
 *
 * Contains info about an annotation on a data frame.
 */
class Annotation {
public:
  class Circle {
  public:
    Circle() {}
    Circle(float r, float x, float y) : r(r), x(x), y(y) {}

    float r;
    float x;
    float y;

    inline bool operator==(const Circle& other) const { return r == other.r && x == other.x && y == other.y; }
  };

  class Polygon {
  public:
    Polygon() {}
    Polygon(std::vector<Vector2<>> points) : points(points) {}

    std::vector<Vector2<>> points;

    inline bool operator==(const Polygon& other) const { return points == other.points; }
  };

  typedef boost::variant<Circle, Polygon> AnnotationInfo;

  Annotation(DataFrame* frame, AnnotationInfo info) : frame(frame), info(info) {}

  Annotation(const StreamableCircleAnnotation&, const std::unordered_map<int, DataFrame*>&);
  Annotation(const StreamablePolygonAnnotation&, const std::unordered_map<int, DataFrame*>&);

  DataFrame* frame;
  AnnotationInfo info;

  inline bool operator==(const Annotation& other) const { return frame == other.frame && info == other.info; }
};

Q_DECLARE_METATYPE(Annotation*);

STREAMABLE(StreamableAnnotation, {
protected:
  StreamableAnnotation(int id, const Annotation&, const std::unordered_map<DataFrame*, int>&);

public:
  ,
    (int)id,      // Annotation index
    (int)frameId, // DataFrame index
});

STREAMABLE_WITH_BASE(StreamableCircleAnnotation, StreamableAnnotation, {
  public : StreamableCircleAnnotation(int id, const Annotation&, const std::unordered_map<DataFrame*, int>&),

  (float)r,
  (float)x,
  (float)y,
});

STREAMABLE_WITH_BASE(StreamablePolygonAnnotation, StreamableAnnotation, {
  public : StreamablePolygonAnnotation(int id, const Annotation&, const std::unordered_map<DataFrame*, int>&),

  (std::vector<Vector2<>>)points,
});
