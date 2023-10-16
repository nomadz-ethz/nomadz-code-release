/**
 * @file VideoView.cpp
 *
 * Implementation of class VideoView
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include <iostream>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QPainter>
#include <QTime>
#include <QVBoxLayout>
#include "Core/System/File.h"
#include "Core/System/Thread.h"
#include "VideoView.h"

VideoWidget::VideoWidget(VideoView& videoView)
    : videoView(videoView), externalTime(0), shift(0), synchronized(true), zoom(1.f), timeMode(CountUp) {
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);

  QVBoxLayout* layout = new QVBoxLayout(this);

  player = new VideoPlayer(this);
  {
    layout->addWidget(player);
    connect(player, SIGNAL(progressed(qint64, qint64)), this, SLOT(updateControls(qint64, qint64)));
  }

  QHBoxLayout* controls = new QHBoxLayout();
  {

    syncBtn = new QPushButton("Sync", this);
    {
      syncBtn->setCheckable(true);
      syncBtn->setChecked(synchronized);
      syncBtn->setEnabled(false);
      controls->addWidget(syncBtn);
      connect(player, SIGNAL(loaded(bool)), syncBtn, SLOT(setEnabled(bool)));
      connect(this, SIGNAL(syncChanged(bool)), syncBtn, SLOT(setChecked(bool)));
      connect(syncBtn, SIGNAL(clicked()), this, SLOT(toggleSync()));
    }

    seekSlider = new QSlider(Qt::Horizontal, this);
    {
      // Units are in ms
      seekSlider->setMaximum(0);
      seekSlider->setMinimum(0);
      seekSlider->setPageStep(3000);
      seekSlider->setSingleStep(300);
      seekSlider->setValue(0);
      controls->addWidget(seekSlider);
      connect(seekSlider, SIGNAL(sliderPressed()), this, SLOT(temporaryPause()));
      connect(seekSlider, SIGNAL(sliderReleased()), this, SLOT(temporaryResume()));
      connect(seekSlider, SIGNAL(valueChanged(int)), this, SLOT(slide(int)));
    }

    timeLabel = new QPushButton("--:--", this);
    {
      timeLabel->setFlat(true);
      controls->addWidget(timeLabel);
      connect(timeLabel, SIGNAL(clicked()), this, SLOT(cycleTimeModes()));
    }

    loadBtn = new QPushButton("Load", this);
    {
      controls->addWidget(loadBtn);
      connect(loadBtn, SIGNAL(clicked()), this, SLOT(loadVideo()));
    }

    layout->addLayout(controls);
  }

  this->setLayout(layout);
}

void VideoWidget::cycleTimeModes() {
  const std::map<TimeMode, TimeMode> nextMode = {{CountUp, CountDown}, {CountDown, CountBoth}, {CountBoth, CountUp}};
  timeMode = nextMode.at(timeMode);

  updateControls(player->currentTime(), player->totalTime());
}

void VideoWidget::loadVideo() {
  std::string dirname = std::string(File::getBHDir());
  QString filename = QFileDialog::getOpenFileName(this, tr("Load Video"), dirname.c_str(), tr("Video files (*.*)"));
  if (filename.isEmpty()) {
    return;
  }

  player->load(filename);
  resync(player->currentTime());
}

void VideoWidget::resync(int videoTime) {
  shift = externalTime - videoTime;
}

void VideoWidget::toggleSync() {
  if (player->isLoaded()) {
    if (synchronized) {
      // Pause
      synchronized = false;
    } else {
      // Play
      synchronized = true;
      resync(player->currentTime());
    }
    emit syncChanged(synchronized);
  }
}

void VideoWidget::slide(int value) {
  resync(value);
  player->seek(value);
}

void VideoWidget::temporaryPause() {
  wasSynchronized = synchronized;
  if (synchronized != false) {
    synchronized = false;
    emit syncChanged(synchronized);
  }
}

void VideoWidget::temporaryResume() {
  if (synchronized != wasSynchronized) {
    synchronized = wasSynchronized;
    emit syncChanged(synchronized);
  }
}

// Doesn't do anything yet
void VideoWidget::wheelEvent(QWheelEvent* event) {
  QWidget::wheelEvent(event);

  zoom += 0.1 * event->delta() / 120;
  if (zoom > 16.f) {
    zoom = 16.f;
  } else if (zoom < 0.1f) {
    zoom = 0.1f;
  }
  QWidget::update();
}

// Doesn't do anything yet
void VideoWidget::mouseDoubleClickEvent(QMouseEvent* event) {
  QWidget::mouseDoubleClickEvent(event);
  zoom = 1;
  QWidget::update();
}

void VideoWidget::update() {
  if ((signed)videoView.externalTime != externalTime) {

    externalTime = videoView.externalTime;

    if (player->isLoaded() && synchronized) {
      // Initialize shift on first non-zero global timestamp
      if (externalTime == 0 && videoView.externalTime != 0) {
        resync(player->currentTime());
      }

      player->seek(externalTime - shift);
    }
  }

  QWidget::update();
}

QString VideoWidget::timeString(qint64 ms) {
  QTime time;
  time = time.addMSecs(ms);
  return (time.hour() > 0) ? time.toString("hh:mm:ss.zzz") : time.toString("mm:ss.zzz");
}

void VideoWidget::updateControls(qint64 current, qint64 total) {
  // Update slider position
  {
    const bool wasBlocked = seekSlider->blockSignals(true);
    if (seekSlider->maximum() != total) {
      seekSlider->setMaximum(total);
    }
    if (seekSlider->value() != current) {
      seekSlider->setValue(current);
    }
    seekSlider->blockSignals(wasBlocked);
  }

  // Update time label
  if (!player->isLoaded()) {
    switch (timeMode) {
    case CountUp:
    case CountDown:
      timeLabel->setText("--:--");
      break;
    case CountBoth:
      timeLabel->setText("--:-- / --:--");
      break;
    }
  } else {
    switch (timeMode) {
    case CountUp:
      timeLabel->setText(timeString(current));
      break;
    case CountDown:
      timeLabel->setText(QString("-") + timeString(total - current));
      break;
    case CountBoth:
      timeLabel->setText(timeString(current) + " / " + timeString(total));
      break;
    }
  }
}

VideoView::VideoView(const QString& fullName, RobotConsole& console, const std::string& name, const int& externalTime)
    : fullName(fullName), icon(":/Icons/tag_green.png"), console(console), name(name), externalTime(externalTime) {}

SimRobot::Widget* VideoView::createWidget() {
  return new VideoWidget(*this);
}
