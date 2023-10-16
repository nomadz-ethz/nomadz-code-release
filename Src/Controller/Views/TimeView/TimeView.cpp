/**
 * @file TimeView.cpp
 *
 * Implementation of class TimeView
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are Arne Böckmann and Colin Graf
 */

#include <QString>
#include "TimeView.h"
#include "Core/System/Thread.h"
#include "Controller/RobotConsole.h"
#include "TimeWidget.h"

TimeView::TimeView(const QString& fullName, RobotConsole& console, const TimeInfo& info)
    : fullName(fullName), icon(":/Icons/tag_green.png"), console(console), info(info) {}

SimRobot::Widget* TimeView::createWidget() {
  return new TimeWidget(*this);
}
