/**
 * @file PixmapTile.h
 *
 * Declaration of class PixmapTile.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <iostream>
#include <QGraphicsItem>
#include <QGraphicsPixmapItem>
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QWidget>
#include <opencv2/core/core.hpp>

#include "Representations/Infrastructure/Image.h"

class PixmapTile : public QGraphicsPixmapItem {
public:
  enum { Type = QGraphicsItem::UserType + 131 };

  PixmapTile(const QPixmap& pixmap, QGraphicsItem* parent = 0) : QGraphicsPixmapItem(pixmap, parent) {}

  // Constructs a new PixmapTile that contains the provided image, with conversion (~-cb-y-cr) -> (r-g-b)
  static PixmapTile* fromBHImage(const Image* image, QGraphicsItem* parent = 0, bool useCache = true);

  // Constructs a new PixmapTile from the given cv::Mat
  static PixmapTile* fromCVMat(const cv::Mat& mat, QGraphicsItem* parent = 0, QString cacheKey = "");

  cv::Mat toMat() const;

  virtual int type() const { return Type; }
};
