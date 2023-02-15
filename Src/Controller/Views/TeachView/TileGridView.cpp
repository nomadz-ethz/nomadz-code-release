/**
 * @file TileGridView.cpp
 *
 * Declaration of class TileGridView, which shows a grid of Tiles.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <cmath>
#include <vector>
#include "Controller/Representations/Dataset.h"
#include "TileGridView.h"

void TileGridView::paintEvent(QPaintEvent* event) {
  // Get list of tiles
  std::vector<PixmapTile*> tiles;
  for (QGraphicsItem* i : scene()->items(Qt::AscendingOrder)) {
    if (i->type() != PixmapTile::Type) {
      continue;
    }
    tiles.push_back(dynamic_cast<PixmapTile*>(i));
  }

  const qreal tileW = tileWidth();
  const qreal tileH = tileHeight();

  for (int i = 0, n = tiles.size(); i < n; ++i) {
    PixmapTile* tile = tiles.at(i);

    const int row = i / columns();
    const int col = i % columns();

    // TODO Center tiles with aspect ratios that don't fit (currently they align top-left)
    const QPointF newPos(spacing() + col * (tileW + spacing()), spacing() + row * (tileH + spacing()));
    if ((tile->pos() - newPos).manhattanLength() >= 0.5) {
      tile->setPos(newPos);
    }

    const qreal initialW = tile->pixmap().width();
    const qreal initialH = tile->pixmap().height();

    const qreal newScale(std::min(tileW / initialW, tileH / initialH));
    if (std::abs(tile->scale() - newScale) >= 1e-6) {
      tile->setScale(newScale);
    }
  }

  const int rows = (int)std::ceil(tiles.size() / (qreal)columns());
  setSceneRect(0, 0, viewport()->width(), rows * (tileH + spacing()) + spacing());

  QGraphicsView::paintEvent(event);
}

void TileGridView::resizeEvent(QResizeEvent* event) {
  const qreal resizeScale = (qreal)event->size().width() / (qreal)event->oldSize().width();
  const QPointF oldCenter(0, visibleSceneRect().y() + event->oldSize().height() / 2);
  centerOn(oldCenter * resizeScale);

  QGraphicsView::resizeEvent(event);
}

int TileGridView::centerTile() const {
  const QRectF viewableRect = visibleSceneRect();
  const QPointF pxCoords(viewableRect.x() + viewableRect.width() / 2, viewableRect.y() + viewableRect.height() / 2);

  const QPoint colRow((int)std::floor((pxCoords.x() - spacing()) / (tileWidth() + spacing())),
                      (int)std::floor((pxCoords.y() - spacing()) / (tileHeight() + spacing())));

  return colRow.x() + columns() * colRow.y();
}

void TileGridView::centerOnTile(int tile) {
  const QPoint colRow(tile % columns(), tile / columns());

  const QPointF pxCoords(colRow.x() * (tileWidth() + spacing()) + spacing(),
                         colRow.y() * (tileHeight() + spacing()) + spacing());

  if (sceneRect().height() < pxCoords.y() + viewport()->height() / 2) {
    setSceneRect(0, 0, sceneRect().width(), pxCoords.y() + viewport()->height() / 2);
  }

  centerOn(pxCoords);
}

void TileGridView::setColumns(int columns) {
  const int center = centerTile();

  _columns = columns;

  centerOnTile(center);
  viewport()->update();
}

std::pair<int, int> TileGridView::tilesInSceneRect(QRectF rect) const {
  const qreal tileH = tileHeight();

  return std::make_pair((int)std::floor(rect.y() / (tileH + spacing())) * columns(),
                        (int)std::ceil((rect.y() + rect.height()) / (tileH + spacing())) * columns() - 1);
}

inline qreal TileGridView::tileWidth() const {
  return (viewport()->width() - (columns() + 1) * spacing()) / columns();
}

inline qreal TileGridView::tileHeight() const {
  return tileWidth() / _aspectRatio;
}
