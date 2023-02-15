/**
 * @file Annotation.cpp
 *
 * Implementation of class Annotation
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "Annotation.h"

Annotation::Annotation(const StreamableCircleAnnotation& annotation, const std::unordered_map<int, DataFrame*>& idToFrame)
    : info(Annotation::Circle(annotation.r, annotation.x, annotation.y)) {
  frame = idToFrame.at(annotation.frameId);
}

Annotation::Annotation(const StreamablePolygonAnnotation& annotation, const std::unordered_map<int, DataFrame*>& idToFrame)
    : info(Annotation::Polygon(annotation.points)) {
  frame = idToFrame.at(annotation.frameId);
}

StreamableAnnotation::StreamableAnnotation(int id,
                                           const Annotation& annotation,
                                           const std::unordered_map<DataFrame*, int>& frameToId)
    : id(id) {
  frameId = frameToId.at(annotation.frame);
}

StreamableCircleAnnotation::StreamableCircleAnnotation(int id,
                                                       const Annotation& annotation,
                                                       const std::unordered_map<DataFrame*, int>& frameToId)
    : StreamableAnnotation(id, annotation, frameToId) {
  const Annotation::Circle& c = boost::get<Annotation::Circle>(annotation.info);
  r = c.r;
  x = c.x;
  y = c.y;
}

StreamablePolygonAnnotation::StreamablePolygonAnnotation(int id,
                                                         const Annotation& annotation,
                                                         const std::unordered_map<DataFrame*, int>& frameToId)
    : StreamableAnnotation(id, annotation, frameToId) {
  const Annotation::Polygon& p = boost::get<Annotation::Polygon>(annotation.info);
  points = p.points;
}
