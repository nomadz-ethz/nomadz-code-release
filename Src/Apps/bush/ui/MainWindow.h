#pragma once

#include <QMainWindow>
#include <QSplitter>
#include <QDesktopWidget>
#include <QSettings>

class TeamSelector;
class ShortcutBar;
class Console;
class RobotPool;

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow();
  ~MainWindow();

protected:
  void closeEvent(QCloseEvent* event);

private:
  TeamSelector* teamSelector;
  ShortcutBar* shortcutBar;
  Console* console;
  RobotPool* robotPool;
  QSplitter* splitter;
  QSplitter* hSplitter;
  QDesktopWidget* desktop;
  QSettings windowSettings;
public slots:
  void saveConfiguration();
};
