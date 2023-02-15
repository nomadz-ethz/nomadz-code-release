/**
 * @file VideoPlayer.cpp
 *
 * Declaration of class VideoPlayer, inspired by QT's Phonon/VideoPlayer.
 * Does not provide automatic playback or audio; can only seek and show video.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <QImage>
#include <QTime>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "VideoPlayer.h"

#if CV_MAJOR_VERSION >= 4
#ifndef CV_CAP_PROP_FPS
#define CV_CAP_PROP_FPS cv::CAP_PROP_FPS
#endif
#ifndef CV_CAP_PROP_POS_MSEC
#define CV_CAP_PROP_POS_MSEC cv::CAP_PROP_POS_MSEC
#endif
#ifndef CV_CAP_PROP_POS_FRAMES
#define CV_CAP_PROP_POS_FRAMES cv::CAP_PROP_POS_FRAMES
#endif
#ifndef CV_CAP_PROP_FRAME_COUNT
#define CV_CAP_PROP_FRAME_COUNT cv::CAP_PROP_FRAME_COUNT
#endif
#endif

VideoPlayer::VideoPlayer(QWidget* parent) : QWidget(parent), captureTime(0), captureFrame(0), frameCount(0), fps(0), dt(0) {}

void VideoPlayer::load(const QString& path) {
  videoCapture.open(path.toUtf8().constData());

  if (readNext()) {
    frameCount = (qint64)videoCapture.get(CV_CAP_PROP_FRAME_COUNT);
    fps = videoCapture.get(CV_CAP_PROP_FPS);
    dt = (int)(1000 / fps);

    update();

    emit loaded(true);
    emit progressed(currentTime(), totalTime());

  } else {
    emit loaded(false);
  }
}

void VideoPlayer::paintEvent(QPaintEvent* event) {
  QImage image(static_cast<unsigned char*>(frame.data), frame.cols, frame.rows, frame.step, QImage::Format_RGB888);

  if (image.width() * image.height() != 0) {
    painter.begin(this);

    // Fit video within widget with uniform scaling
    if (image.width() * height() > image.height() * width()) {
      // Video wider than widget -> letterbox on top & bottom
      const int excessHeight = height() - image.height() * width() / image.width();
      painter.drawImage(QRectF(0, excessHeight / 2, width(), height() - excessHeight),
                        image.rgbSwapped(),
                        QRectF(0, 0, image.width(), image.height()));

    } else {
      // Video taller than widget -> letterbox on left & right
      const int excessWidth = width() - image.width() * height() / image.height();
      painter.drawImage(QRectF(excessWidth / 2, 0, width() - excessWidth, height()),
                        image.rgbSwapped(),
                        QRectF(0, 0, image.width(), image.height()));
    }

    painter.end();
  }

  QWidget::paintEvent(event);
}

void VideoPlayer::seek(qint64 ms) {
  qint64 frame = ms / dt;
  if (frame < 0) {
    frame = 0;
    ms = 0;
  } else if (frame >= frameCount) {
    frame = frameCount - 1;
    ms = frame * dt;
  }

  qint64 seekDifference = frame - captureFrame;

  if (seekDifference <= -2 || seekDifference >= 8) {
    videoCapture.set(CV_CAP_PROP_POS_MSEC, (double)ms);

    if (readNext()) {
      update();
      emit progressed(currentTime(), totalTime());
    }

  } else if (seekDifference >= 0) {
    while (seekDifference >= 1) {
      --seekDifference;
      videoCapture.grab();
    }

    if (readNext()) {
      update();
      emit progressed(currentTime(), totalTime());
    }
  }
}

bool VideoPlayer::readNext() {
  if (!videoCapture.isOpened()) {
    return false;
  }

  cv::Mat frame2;
  if (videoCapture.read(frame2)) {
    std::swap(frame, frame2);

    captureTime = (qint64)videoCapture.get(CV_CAP_PROP_POS_MSEC);
    captureFrame = (qint64)videoCapture.get(CV_CAP_PROP_POS_FRAMES);
    return true;

  } else {
    return false;
  }
}
