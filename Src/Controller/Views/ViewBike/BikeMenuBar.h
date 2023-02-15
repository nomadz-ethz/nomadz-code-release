/**
 * @file BikeMenuBar.h
 *
 * Declaration of class BikeMenuBar
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:judy@tzi.de">Judith Müller</a>
 */

#pragma once

#include <QMenuBar>
#include <QWidget>

class BikeMenuBar : public QMenuBar {
public:
  BikeMenuBar();

  QAction* xy_plane;
  QAction* xz_plane;
  QAction* yz_plane;

  QMenu* phaseMenu;
  QMenu* dragPlaneMenu;

  QActionGroup* dragPlaneActionGroup;

private:
  void createActions();
  void createMenus();
};
