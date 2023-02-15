/**
 * @file AnnotationItem.cpp
 *
 * Implementation of classes CircleAnnotationItem & PolygonAnnotationItem.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <list>
#include <QPainterPathStroker>
#include <QPointF>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "AnnotationItem.h"

void AnnotationItem::mousePressEvent(QGraphicsSceneMouseEvent* event) {
  std::list<QGraphicsItem*> stackBehind; // stack these behind this
  std::list<QGraphicsItem*> stackAbove;  // stack these above this

  for (QGraphicsItem* item : collidingItems()) {
    if (item->type() != CircleAnnotationItem::Type && item->type() != PolygonAnnotationItem::Type) {
      continue;
    }

    if (item->collidesWithItem(this, Qt::ContainsItemShape)) {
      stackAbove.push_back(item);
    } else {
      stackBehind.push_back(item);
    }
  }

  for (QGraphicsItem* item : stackBehind) {
    item->stackBefore(this);
  }

  for (QGraphicsItem* item : stackAbove) {
    this->stackBefore(item);
  }
}

qreal CircleAnnotationItem::actualThickness() const {
  qreal th = 5.0 / std::sqrt(colors().size());
  if (colors().size() * th > circle.r) {
    th = circle.r / (qreal)colors().size();
  }
  return th;
}

QRectF CircleAnnotationItem::boundingRect() const {
  const qreal th = actualThickness();
  return QRectF(circle.x - circle.r - th / 2, circle.y - circle.r - th / 2, 2 * circle.r + th, 2 * circle.r + th);
}

bool CircleAnnotationItem::contains(const QPointF& point) const {
  const qreal th = actualThickness();

  const qreal dx = point.x() - circle.x;
  const qreal dy = point.y() - circle.y;
  const qreal r = circle.r + th / 2;

  return dx * dx + dy * dy <= r * r;
}

void CircleAnnotationItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) {
  const bool selected = option->state & QStyle::State_Selected;

  painter->setRenderHints(QPainter::Antialiasing);
  painter->setBrush(Qt::NoBrush);

  const qreal th = actualThickness();
  QPen pen;
  pen.setWidthF(th);

  for (int i = 0; i < colors().size(); ++i) {
    const qreal r = circle.r + (i - colors().size() + 1) * th;

    QColor color(colors()[i]);
    if (!selected) {
      color.setAlpha(100);
    }
    pen.setColor(color);

    painter->setPen(pen);
    painter->drawEllipse(QRectF(circle.x - r, circle.y - r, 2 * r, 2 * r));
  }
}

QPainterPath CircleAnnotationItem::shape() const {
  const qreal th = actualThickness();

  QPainterPath path;
  path.addEllipse(circle.x - circle.r - th / 2, circle.y - circle.r - th / 2, 2 * circle.r + th, 2 * circle.r + th);
  return path;
}

qreal PolygonAnnotationItem::actualThickness() const {
  return thickness() / std::sqrt(colors().size());
}

QRectF PolygonAnnotationItem::boundingRect() const {
  const qreal extra = actualThickness() * (colors().size() - 0.5);
  return polygon.boundingRect().adjusted(-extra, -extra, extra, extra);
}

bool PolygonAnnotationItem::contains(const QPointF& point) const {
  return polygons().back().containsPoint(point, Qt::OddEvenFill);
}

void PolygonAnnotationItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) {
  const bool selected = (option->state & QStyle::State_Selected);

  painter->setRenderHints(QPainter::Antialiasing);
  painter->setBrush(Qt::NoBrush);

  QPen pen;
  pen.setJoinStyle(Qt::MiterJoin);
  pen.setWidthF(actualThickness());

  for (int i = 0; i < colors().size(); ++i) {
    QColor color(colors()[i]);
    if (!selected) {
      color.setAlpha(100);
    }
    pen.setColor(color);
    painter->setPen(pen);

    painter->drawPolygon(polygons()[i]);
  }
}

QPainterPath PolygonAnnotationItem::shape() const {
  QPainterPath path;
  path.addPolygon(polygons().back());
  return path;
}

void PolygonAnnotationItem::updatePolygons() const {
  _polygons.clear();
  _polygons.push_back(polygon);

  // Set up how much to expand each time
  std::vector<qreal> ths;
  {
    const qreal th = actualThickness();
    for (int i = 1; i < colors().size(); ++i) {
      ths.push_back(th);
    }
    ths.push_back(th / 2); // "outer shell": add pen thickness
  }

  if (polygon.size() > 2) {
    // Iteratively expand original polygon
    // TODO Expand polygon
    QPolygonF expanded = polygon;
    for (size_t i = 0; i < ths.size(); ++i) {
      QPolygonF polygon2 = expanded;
      _polygons.push_back(polygon2);
    }

  } else if (polygon.size() == 2) {
    // Iteratively expand original line
    // TODO Expand rectangular approximation of line
    QPolygonF expanded = polygon;
    for (size_t i = 0; i < ths.size(); ++i) {
      QPolygonF polygon2 = expanded;
      _polygons.push_back(polygon2);
    }
  }

  polygonsNeedUpdate = false;
}
