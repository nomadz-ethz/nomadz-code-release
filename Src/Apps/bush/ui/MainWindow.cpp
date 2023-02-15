#include "Core/System/File.h"
#include <QBoxLayout>
#include <QFileDialog>
#include <QGridLayout>
#include <QLabel>
#include <QMenuBar>
#include <QPixmap>
#include <QPushButton>
#include "ui/Console.h"
#include "ui/MainWindow.h"
#include "ui/RobotPool.h"
#include "ui/ShortcutBar.h"
#include "ui/TeamSelector.h"
#include "Session.h"

MainWindow::MainWindow()
    : teamSelector(new TeamSelector()), shortcutBar(0), console(new Console(teamSelector)), robotPool(0),
      splitter(new QSplitter(Qt::Vertical)), hSplitter(new QSplitter(Qt::Horizontal)), windowSettings("NomadZ", "bush") {
  splitter->addWidget(teamSelector);
  splitter->addWidget(console);
  splitter->setStretchFactor(0, 0);

  shortcutBar = new ShortcutBar(console);
  addToolBar(Qt::BottomToolBarArea, shortcutBar);
  shortcutBar->addShortcut("help", "help", "help");
  shortcutBar->addShortcut("ping", "ping", "network");
  shortcutBar->addShortcut("deploy", "deploy", "browser-download");
  shortcutBar->addShortcut("restart", "restart", "none");
  shortcutBar->addShortcut("SimRobot", "sim", "none");
  shortcutBar->addShortcut("download logs", "downloadLogs", "download logs");
  shortcutBar->addShortcut("update wireless profiles", "updateWireless", "none");
  shortcutBar->addShortcut("update settings", "updateSettings", "none");

  hSplitter->addWidget(splitter);
  robotPool = new RobotPool(teamSelector);
  connect(teamSelector, SIGNAL(currentChanged(int)), robotPool, SLOT(update()));
  QFrame* rightSide = new QFrame(this);
  QGridLayout* rsLayout = new QGridLayout(rightSide);
  rsLayout->addWidget(new QLabel("<b>Robot Pool:</b>"), 0, 0);
  rsLayout->addWidget(robotPool, 1, 0);
  connect(teamSelector->saveConfigButton, SIGNAL(clicked(bool)), this, SLOT(saveConfiguration()));
  connect(shortcutBar, SIGNAL(actionTriggered(QAction*)), this, SLOT(saveConfiguration()));
  rightSide->setLayout(rsLayout);
  rightSide->setMaximumWidth(190);
  hSplitter->addWidget(rightSide);
  setCentralWidget(hSplitter);

  this->setWindowIcon(QPixmap(":icons/bush.png"));
  console->setFocus(Qt::OtherFocusReason);
  console->resize(60, 500);

  teamSelector->loadTeams();
  int widgetWidth = 1008;
  int widgetHeight = 700;
  splitter->setMinimumWidth(200);
  QWidget::setMinimumHeight(widgetHeight);
  QWidget::setMinimumWidth(widgetWidth);
  QWidget::resize(widgetWidth, widgetHeight);
  desktop = new QDesktopWidget();
  QPoint position((desktop->width() - frameGeometry().width()) / 2, (desktop->height() - frameGeometry().height()) / 2);
  QWidget::move(position);

  restoreGeometry(windowSettings.value("mainWindow/geometry").toByteArray());

  Session::getInstance().setMainWindow(this);
}

MainWindow::~MainWindow() {
  delete console;
  delete splitter;
  delete desktop;
}

void MainWindow::closeEvent(QCloseEvent* event) {
  windowSettings.setValue("mainWindow/geometry", saveGeometry());
  QMainWindow::closeEvent(event);
}

void MainWindow::saveConfiguration() {
  Session::getInstance().saveRobotsConfiguration();
  teamSelector->saveTeams();
}
