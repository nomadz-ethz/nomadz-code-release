/**
 * @file PixmapTile.cpp
 *
 * Implementation of class PixmapTile.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <QPixmapCache>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "PixmapTile.h"

PixmapTile* PixmapTile::fromBHImage(const Image* image, QGraphicsItem* parent, bool useCache) {

  QPixmap pixmap;

  // If useCache and cached pixmap exists, use that
  if (useCache) {
    QString pixmapKey = std::to_string(reinterpret_cast<uintptr_t>(image)).c_str();
    if (QPixmapCache::find(pixmapKey, &pixmap)) {
      return new PixmapTile(pixmap, parent);
    }
  }

  // Otherwise, make a pixmap the hard way
  {
    cv::Mat inMat(image->height,
                  image->width,
                  CV_8UC4,
                  (void*)&((*image)[0][0].channels[0]),
                  image->widthStep * sizeof(decltype(image->widthStep)));
    cv::Mat mat(inMat.rows, inMat.cols, CV_8UC3);

    cv::mixChannels({inMat},
                    {mat},
                    {
                      2,
                      0, // Pixel.y -> out[0]
                      3,
                      1, // Pixel.cr -> out[1]
                      1,
                      2 // Pixel.cb -> out[2]
                    });
    cv::cvtColor(mat, mat, cv::COLOR_YCrCb2RGB);

    QImage qimage(static_cast<unsigned char*>(mat.data), mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
    pixmap = QPixmap::fromImage(qimage);
  }

  // If useCache, store it
  if (useCache) {
    QString pixmapKey = std::to_string(reinterpret_cast<uintptr_t>(image)).c_str();
    QPixmapCache::insert(pixmapKey, pixmap);
  }

  return new PixmapTile(pixmap, parent);
}

PixmapTile* PixmapTile::fromCVMat(const cv::Mat& mat, QGraphicsItem* parent, QString cacheKey) {

  QPixmap pixmap;

  const bool useCache = !cacheKey.isEmpty();

  // If useCache and cached pixmap exists, use that
  if (useCache) {
    if (QPixmapCache::find(cacheKey, &pixmap)) {
      return new PixmapTile(pixmap, parent);
    }
  }

  // Otherwise, make a pixmap
  {
    QImage qimage(static_cast<unsigned char*>(mat.data), mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
    pixmap = QPixmap::fromImage(qimage);
  }

  if (useCache) {
    QPixmapCache::insert(cacheKey, pixmap);
  }

  return new PixmapTile(pixmap, parent);
}

cv::Mat PixmapTile::toMat() const {
  QImage img = pixmap().toImage();
  cv::Mat mat(img.height(), img.width(), CV_8UC4, img.bits(), img.bytesPerLine());
  cv::cvtColor(mat, mat, cv::COLOR_BGRA2BGR); // drop alpha
  return mat;
}
