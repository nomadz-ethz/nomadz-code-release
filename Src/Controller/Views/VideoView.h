/**
 * @file VideoView.h
 *
 * Declaration of class VideoView
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <QString>
#include <QIcon>
#include <QMouseEvent>
#include <QPushButton>
#include <QWidget>
#include "Controller/RoboCupCtrl.h"
#include "Controller/RobotConsole.h"
#include "Controller/Visualization/VideoPlayer.h"
#include "SimRobot.h"

class RobotConsole;

/**
 * @class VideoView
 *
 * A class to represent a view displaying a video player.
 */
class VideoView : public SimRobot::Object {
public:
  /**
   * Constructor.
   * @param fullName The path to this view in the scene graph
   * @param console The console object.
   * @param name The name of the view.
   */
  VideoView(const QString& fullName, RobotConsole& console, const std::string& name, const int& externalTime);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon;       /**< The icon used for listing this view in the scene graph */
  RobotConsole& console;  /**< A reference to the console object. */
  const std::string name; /**< The name of the view. */

  const int& externalTime; /**< A reference to the time this view synchronizes with */

  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const { return fullName; }
  virtual const QIcon* getIcon() const { return &icon; }

  friend class VideoWidget;
};

class VideoWidget : public QWidget, public SimRobot::Widget {
public:
  VideoWidget(VideoView& videoView);
  virtual QWidget* getWidget() { return this; }

signals:
  void syncChanged(bool);

private slots:
  void cycleTimeModes();
  void loadVideo();
  void resync(int);
  void toggleSync();
  void slide(int);
  void temporaryPause();
  void temporaryResume();
  void updateControls(qint64, qint64);

private:
  Q_OBJECT

  VideoView& videoView;
  int externalTime;
  int shift; // video time = sync time - shift
  bool synchronized;
  float zoom;
  VideoPlayer* player;
  QPushButton* syncBtn;
  QSlider* seekSlider;
  QPushButton* timeLabel;
  QPushButton* loadBtn;

  bool wasSynchronized;

  enum TimeMode { CountUp, CountDown, CountBoth };
  TimeMode timeMode;

  QString timeString(qint64 ms);

  // Doesn't do anything yet
  void wheelEvent(QWheelEvent* event);

  // Doesn't do anything yet
  void mouseDoubleClickEvent(QMouseEvent*);

  void update();
};
