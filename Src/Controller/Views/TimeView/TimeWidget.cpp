/**
 * @file TimeWidget.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Arne Böckmann
 */

#include "TimeWidget.h"
#include <QtGlobal>
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
#define setSectionResizeMode setResizeMode
#endif
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include "TimeView.h"
#include "Core/System/Thread.h"
#include "Core/System/Time.h"
#include "Controller/RobotConsole.h"

/**A simple QTableWidgetItem that enables correct sorting of numbers*/
class NumberTableWidgetItem : public QTableWidgetItem {
  bool operator<(const QTableWidgetItem& other) const override { return this->text().toFloat() < other.text().toFloat(); }
};

struct Row {
  QTableWidgetItem* name; // QTableWidgetItem uses lexical sorting, that is ok for the name column
  NumberTableWidgetItem* min;
  NumberTableWidgetItem* max;
  NumberTableWidgetItem* avg;
};

TimeWidget::TimeWidget(TimeView& timeView) : timeView(timeView), lastTimeInfoTimeStamp(0) {
  table = new QTableWidget();
  table->setColumnCount(4);
  QStringList headerNames;
  headerNames << "Stopwatch"
              << "Min"
              << "Max"
              << "Avg";
  table->setHorizontalHeaderLabels(headerNames);
  table->horizontalHeader()->resizeSections(QHeaderView::ResizeToContents);
  table->horizontalHeader()->resizeSection(0, 200);
  table->verticalHeader()->setVisible(false);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
  table->setAlternatingRowColors(true);
  table->setSortingEnabled(true);
  table->sortItems(3, Qt::DescendingOrder); // initial sort by avg
  QVBoxLayout* layout = new QVBoxLayout(this);
  QHBoxLayout* filterLayout = new QHBoxLayout();
  filterLayout->addWidget(new QLabel("Filter:"));
  QLineEdit* filterEdit = new QLineEdit();
  filterEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  filterLayout->addWidget(filterEdit);
  frequency = new QLabel("Frequency: ");
  layout->addWidget(frequency);
  layout->addLayout(filterLayout);
  layout->addWidget(table);
  this->setLayout(layout);
  lastTimeInfoTimeStamp = Time::getCurrentSystemTime();
  lastUpdate = Time::getCurrentSystemTime();
  QObject::connect(filterEdit, SIGNAL(textChanged(QString)), this, SLOT(filterChanged(QString)));
}

QWidget* TimeWidget::getWidget() {
  return this;
}
void TimeWidget::update() {
  SYNC_WITH(timeView.console);
  {
    if (timeView.info.timeStamp == lastTimeInfoTimeStamp) {
      return;
    }
    lastTimeInfoTimeStamp = timeView.info.timeStamp;

    if (Time::getTimeSince(lastUpdate) < 200) // only update 5 times per second
    {
      return;
    }
    lastUpdate = Time::getCurrentSystemTime();

    float avgFrequency = -1.0;
    timeView.info.getProcessStatistics(avgFrequency);
    frequency->setText("Frequency: " + QString::number(avgFrequency));

    table->setUpdatesEnabled(false);
    table->setSortingEnabled(false); // disable sorting while updating to avoid race conditions
    for (TimeInfo::Infos::const_iterator i = timeView.info.infos.begin(), end = timeView.info.infos.end(); i != end; ++i) {

      std::string name = timeView.info.getName(i->first);
      Row* currentRow = NULL;
      if (items.find(i->first) != items.end()) { // already know this one
        currentRow = items[i->first];
      } else { // new item
        currentRow = new Row;
        currentRow->avg = new NumberTableWidgetItem();
        currentRow->max = new NumberTableWidgetItem();
        currentRow->min = new NumberTableWidgetItem();
        currentRow->name = new QTableWidgetItem();
        const int rowCount = table->rowCount();
        table->setRowCount(rowCount + 1);
        table->setItem(rowCount, 0, currentRow->name);
        table->setItem(rowCount, 1, currentRow->min);
        table->setItem(rowCount, 2, currentRow->max);
        table->setItem(rowCount, 3, currentRow->avg);
        items[i->first] = currentRow;
      }
      float minTime = -1, maxTime = -1, avgTime = -1;
      timeView.info.getStatistics(i->second, minTime, maxTime, avgTime);
      currentRow->avg->setText(QString::number(avgTime));
      currentRow->min->setText(QString::number(minTime));
      currentRow->max->setText(QString::number(maxTime));
      currentRow->name->setText(QString(name.c_str())); // refresh name every time to eliminate unknown
    }
  }
  applyFilter();
  table->setSortingEnabled(true);
  table->setUpdatesEnabled(true);
  table->update();
}

void TimeWidget::filterChanged(const QString& newFilter) {
  filter = newFilter;
}

void TimeWidget::applyFilter() {
  for (int i = 0; i < table->rowCount(); ++i) {
    QTableWidgetItem* item = table->item(i, 0); // assuming that column 0 is the name column
    if (NULL != item && item->text().contains(filter.trimmed())) {
      table->setRowHidden(i, false);
    } else {
      table->setRowHidden(i, true);
    }
  }
}
