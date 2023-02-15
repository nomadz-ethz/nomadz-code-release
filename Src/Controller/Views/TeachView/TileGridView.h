/**
 * @file TileGridView.h
 *
 * Declaration of class TileGridView, which shows a grid of Tiles.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <iostream>
#include <set>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QList>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QTime>
#include <QWidget>
#include "PixmapTile.h"

class TileGridView : public QGraphicsView {
public:
  TileGridView(QGraphicsScene* scene, QWidget* parent = 0)
      : QGraphicsView(scene, parent), _aspectRatio(640. / 480.), _columns(5), _spacing(20.0) {}

  virtual void paintEvent(QPaintEvent* event);

  virtual void resizeEvent(QResizeEvent* event);

  inline qreal aspectRatio() const { return _aspectRatio; }

  inline int columns() const { return _columns; }

  inline qreal spacing() const { return _spacing; }

  inline qreal tileHeight() const;

  inline qreal tileWidth() const;

  inline QRectF visibleSceneRect() const { return mapToScene(viewport()->geometry()).boundingRect(); }

public slots:
  inline void setAspectRatio(qreal aspectRatio) {
    if (_aspectRatio != aspectRatio) {
      _aspectRatio = aspectRatio;
      viewport()->update();
    }
  }
  void setColumns(int);

  inline void setSpacing(qreal spacing) {
    if (_spacing != spacing) {
      _spacing = spacing;
      viewport()->update();
    }
  }

private:
  Q_OBJECT;

  qreal _aspectRatio;
  int _columns;
  qreal _spacing;

  int centerTile() const;

  void centerOnTile(int);

  std::pair<int, int> tilesInSceneRect(QRectF) const;
};
