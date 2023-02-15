/**
 * @file CameraCalibrationView.h

 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#pragma once

#include "SimRobot.h"
#include "Controller/RobotConsole.h"
#include "RangeSelector.h"
#include "ThresholdSelector.h"

class ColorCalibrationWidget;
class ColorCalibrationView;

class ColorCalibrationView : public SimRobot::Object {
public:
  RobotConsole& console;
  ColorCalibrationWidget* widget;

  ColorCalibrationView(const QString& fullName, RobotConsole& console);
  virtual ~ColorCalibrationView() {}

  virtual SimRobot::Widget* createWidget();
  virtual const QString& getFullName() const;
  virtual const QIcon* getIcon() const;

private:
  const QString fullName;
  const QIcon icon;
};

class ColorCalibrationWidget : public QWidget, public SimRobot::Widget {
  Q_OBJECT

public:
  ColorCalibrationView& colorCalibrationView;
  unsigned int currentColor;

  ColorCalibrationWidget(ColorCalibrationView& colorCalibrationView);
  virtual ~ColorCalibrationWidget();
  virtual QWidget* getWidget();
  virtual void update();
  void updateWidgets(unsigned int currentColor);
  ColorReference* colorReference() const;

private:
  HueSelector* hue;
  SaturationSelector* saturation;
  ValueSelector* value;

  // color class white
  ThresholdSelector* minR;
  ThresholdSelector* minB;
  ThresholdSelector* minRB;

  // color class black
  ThresholdSelector* minI;
};
