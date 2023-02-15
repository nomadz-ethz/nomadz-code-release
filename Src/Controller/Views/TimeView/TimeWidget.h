/**
 * @file TimeWidget.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Arne Böckmann
 */
#pragma once
#include <QObject>
#include <string>
#include <unordered_map>
#include <QWidget>
#include "SimRobot.h"
#include <QString>

struct Row;
class QTableWidget;
class TimeView;
class QLabel;

/**A widget that is used inside a TimeView to display timings*/
class TimeWidget : public QWidget, public SimRobot::Widget {
  Q_OBJECT;

public:
  TimeWidget(TimeView& timeView);

  virtual QWidget* getWidget();
  virtual void update();

private slots:
  void filterChanged(const QString& newFilter);

private:
  /**Hides all rows from the table that fit the filter*/
  void applyFilter();

private:
  QLabel* frequency;
  TimeView& timeView;
  QTableWidget* table; /**< The table that displays the timings */
  unsigned lastTimeInfoTimeStamp;
  std::unordered_map<unsigned short, Row*> items;
  unsigned lastUpdate; /**< time of the last update. Used to manage update rate */
  QString filter;      /**< the current filter that has been entered */
};
