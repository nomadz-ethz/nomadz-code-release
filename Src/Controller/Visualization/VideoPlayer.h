/**
 * @file VideoPlayer.h
 *
 * Declaration of class VideoPlayer, inspired by QT's Phonon/VideoPlayer.
 * Does not provide automatic playback or audio; can only seek and show video.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <iostream>
#include <memory>
#include <QPainter>
#include <QPaintEvent>
#include <QWidget>
#include <opencv2/highgui/highgui.hpp>

class VideoPlayer : public QWidget {
public:
  VideoPlayer(QWidget* parent = nullptr);

  virtual void paintEvent(QPaintEvent* event);

  bool isLoaded() const { return videoCapture.isOpened(); }

  qint64 currentTime() const { return captureTime; }

  qint64 totalTime() const { return frameCount * dt; };

signals:
  void loaded(bool);
  void progressed(qint64, qint64);

public slots:
  void load(const QString& path);

  void seek(qint64 ms);

private:
  Q_OBJECT;

  QPainter painter;
  cv::Mat frame;
  cv::VideoCapture videoCapture;

  qint64 captureTime;  // current time in the videoCapture
  qint64 captureFrame; // current frame in the videoCapture

  qint64 frameCount; // total number of frames in the video
  double fps;        // frames per second in the video
  int dt;            // ms per frame in the video

  bool readNext();
};
