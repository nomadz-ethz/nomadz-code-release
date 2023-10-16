/**
 * @file AnnotationItem.h
 *
 * Declaration of classes CircleAnnotationItem & PolygonAnnotationItem.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <iostream>
#include <QGraphicsEllipseItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPolygonF>
#include <QStyleOptionGraphicsItem>
#include <QVector>

#include "Controller/Representations/Annotation.h"

class AnnotationItem : public QGraphicsItem {
public:
  AnnotationItem(Annotation* annotation, const QVector<QColor>& colors, QGraphicsItem* parent = 0)
      : QGraphicsItem(parent), _annotation(annotation), _colors(colors), _thickness(6.0) {}

  inline Annotation* annotation() const { return _annotation; }

  inline const QVector<QColor>& colors() const { return _colors; }

  inline qreal thickness() const { return _thickness; }

  virtual void setAnnotation(Annotation* annotation) {
    if (_annotation != annotation) {
      prepareGeometryChange();
      _annotation = annotation;
    }
  }

  virtual void setColors(QVector<QColor>& colors) {
    if (_colors.size() != colors.size()) {
      prepareGeometryChange();
    }

    if (_colors != colors) {
      _colors = colors;
    }
  }

  virtual void setThickness(qreal thickness) {
    if (_thickness != thickness) {
      prepareGeometryChange();
      _thickness = thickness;
    }
  }

protected:
  Annotation* _annotation;
  QVector<QColor> _colors;
  qreal _thickness; // standard thickness of one colored circle (may shrink when there are multiple colors)

  virtual void mousePressEvent(QGraphicsSceneMouseEvent* event);
};

class CircleAnnotationItem : public AnnotationItem {
public:
  enum { Type = QGraphicsItem::UserType + 141 };

  CircleAnnotationItem(Annotation* annotation, const QVector<QColor>& colors, QGraphicsItem* parent = 0)
      : AnnotationItem(annotation, colors, parent) {
    setAnnotation(annotation);
  }

  virtual QRectF boundingRect() const;

  virtual bool contains(const QPointF&) const;

  virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* options, QWidget* widget = 0);

  virtual void setAnnotation(Annotation* annotation) {
    AnnotationItem::setAnnotation(annotation);
    circle = boost::get<Annotation::Circle>(_annotation->info);
  }

  virtual QPainterPath shape() const;

  virtual int type() const { return Type; }

private:
  Annotation::Circle circle;

  // Thickness of each individual colored circle
  qreal actualThickness() const;
};

class PolygonAnnotationItem : public AnnotationItem {
public:
  enum { Type = QGraphicsItem::UserType + 142 };

  PolygonAnnotationItem(Annotation* annotation, const QVector<QColor>& colors, QGraphicsItem* parent = 0)
      : AnnotationItem(annotation, colors, parent), polygonsNeedUpdate(true) {
    setAnnotation(annotation);
  }

  virtual QRectF boundingRect() const;

  virtual bool contains(const QPointF&) const;

  virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* options, QWidget* widget = 0);

  virtual void setAnnotation(Annotation* annotation) {
    polygonsNeedUpdate = true;
    AnnotationItem::setAnnotation(annotation);

    polygon.clear();
    for (const auto& pt : boost::get<Annotation::Polygon>(_annotation->info).points) {
      polygon << QPointF(pt.x, pt.y);
    }
  }

  virtual void setColors(QVector<QColor>& colors) {
    if (_colors.size() != colors.size()) {
      polygonsNeedUpdate = true;
    }

    AnnotationItem::setColors(colors);
  }

  virtual void setThickness(qreal thickness) {
    if (_thickness != thickness) {
      polygonsNeedUpdate = true;
    }

    AnnotationItem::setThickness(thickness);
  }

  virtual QPainterPath shape() const;

  virtual int type() const { return Type; }

private:
  QPolygonF polygon; // comes from the annotation

  mutable std::vector<QPolygonF> _polygons; // successively "expanded" polygons, calculated once and cached here
  mutable bool polygonsNeedUpdate;          // polygons vector needs update

  // Thickness of each individual colored polgyon
  qreal actualThickness() const;

  std::vector<QPolygonF>& polygons() const {
    if (polygonsNeedUpdate) {
      updatePolygons();
    }

    return _polygons;
  }

  // Recalculate vector of polygons
  void updatePolygons() const;
};

class AnnotationItemFactory : public boost::static_visitor<AnnotationItem*> {
public:
  AnnotationItemFactory(Annotation* annotation) : annotation(annotation) {}

  AnnotationItemFactory(Annotation* annotation, const QVector<QColor>& colors) : annotation(annotation), colors(colors) {}

  AnnotationItem* operator()(Annotation::Circle c) const { return new CircleAnnotationItem(annotation, colors); }

  AnnotationItem* operator()(Annotation::Polygon p) const { return new PolygonAnnotationItem(annotation, colors); }

private:
  Annotation* annotation;
  QVector<QColor> colors;
};
