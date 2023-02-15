/**
 * @file GroupCheckBox.h
 *
 * Declaration of class GroupCheckBox.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <iostream>
#include <QCheckBox>
#include <QObject>
#include <QWidget>

class GroupCheckBox : public QCheckBox {
public:
  GroupCheckBox(QWidget* parent = 0) : QCheckBox(parent) {}

  GroupCheckBox(const QString& text, QWidget* parent = 0) : QCheckBox(text, parent) {}

  virtual void nextCheckState();

private:
  Q_OBJECT;
};
