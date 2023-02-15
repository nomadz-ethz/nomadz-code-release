/**
 * @file BHToolBar.h
 *
 * B-HumanTool with awesome buttons.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Florian Maaß
 */

#pragma once

#include <QMenu>

class ConsoleRoboCupCtrl;

class BHToolBar : public QObject {
  Q_OBJECT

public:
  BHToolBar(ConsoleRoboCupCtrl& console) : console(console) {}

  QMenu* createUserMenu() const;

private:
  ConsoleRoboCupCtrl& console;

private slots:
  void stand();
  void sitDown();
  void setPlayDead();
  void setStand();
  void setSitDown();
  void pressChestButton();
  void releaseChestButton();
  void unchangeButtons();
};
