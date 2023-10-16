#include "ui/TeamView.h"
#include "models/Team.h"
#include "ui/RobotView.h"
#include "ui/TeamSelector.h"
#include "tools/StringTools.h"
#include "tools/Filesystem.h"

#include <QCheckBox>
#include <QComboBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QLabel>
#include <QSlider>
#include <QEvent>

void TeamView::init() {
  if (team) {
    QFormLayout* layout = new QFormLayout();
    QGridLayout* settingsGrid = new QGridLayout();
    settingsGrid->setColumnStretch(13, 1);

    cbColor1 = new QComboBox(this);
    cbColor1->addItem("red");
    cbColor1->addItem("blue");
    cbColor1->setCurrentIndex(cbColor1->findText(fromString(team->teamColor)));
    settingsGrid->addWidget(new QLabel("Team Color:", cbColor1), 0, 0);
    settingsGrid->addWidget(cbColor1, 0, 1);
    connect(cbColor1, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(teamColorChanged(const QString&)));

    cbColor2 = new QComboBox(this);
    cbColor2->addItem("orange");
    cbColor2->addItem("black");
    cbColor2->setCurrentIndex(cbColor2->findText(fromString(team->keeperColor)));
    settingsGrid->addWidget(new QLabel("Keeper Color:", cbColor2), 1, 0);
    settingsGrid->addWidget(cbColor2, 1, 1);
    connect(cbColor2, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(keeperColorChanged(const QString&)));

    sbNumber = new QSpinBox(this);
    sbNumber->setRange(1, 99);
    sbNumber->setButtonSymbols(QAbstractSpinBox::NoButtons);
    sbNumber->setMaximumWidth(30);
    sbNumber->setValue(team->number);
    settingsGrid->addWidget(new QLabel("Number:", sbNumber), 0, 2);
    settingsGrid->addWidget(sbNumber, 0, 3);
    connect(sbNumber, SIGNAL(valueChanged(int)), this, SLOT(numberChanged(int)));
    sbNumber->installEventFilter(this);

    const int port = 10000 + (team->number);
    setPort(port);

    cbLocation = new QComboBox(this);
    std::vector<std::string> locations = Filesystem::getLocations();
    for (size_t i = 0; i < locations.size(); ++i)
      cbLocation->addItem(fromString(locations[i]));
    cbLocation->setCurrentIndex(cbLocation->findText(fromString(team->location)));
    settingsGrid->addWidget(new QLabel("Location:", lePort), 0, 4);
    settingsGrid->addWidget(cbLocation, 0, 5);
    connect(cbLocation, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(locationChanged(const QString&)));

    cbWlanConfig = new QComboBox(this);
    std::vector<std::string> configs = Filesystem::getWlanConfigs();
    for (size_t i = 0; i < configs.size(); ++i)
      cbWlanConfig->addItem(fromString(configs[i]));
    cbWlanConfig->setCurrentIndex(cbWlanConfig->findText(fromString(team->wlanConfig)));
    settingsGrid->addWidget(new QLabel("Wlan:", cbWlanConfig), 0, 6);
    settingsGrid->addWidget(cbWlanConfig, 0, 7);
    connect(cbWlanConfig, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(wlanConfigChanged(const QString&)));

    cbBuildConfig = new QComboBox(this);
    cbBuildConfig->addItem("Develop");
    cbBuildConfig->addItem("Release");
    cbBuildConfig->addItem("Debug");
    cbBuildConfig->setCurrentIndex(cbBuildConfig->findText(fromString(team->buildConfig)));
    settingsGrid->addWidget(new QLabel("Conf:", cbBuildConfig), 0, 8);
    settingsGrid->addWidget(cbBuildConfig, 0, 9);
    connect(cbBuildConfig, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(buildConfigChanged(const QString&)));

    cbFormation = new QComboBox(this);
    cbFormation->addItem("A");
    cbFormation->addItem("B");
    cbFormation->addItem("C");
    cbFormation->setCurrentIndex(cbFormation->findText(fromString(team->formation)));
    settingsGrid->addWidget(new QLabel("Formation:", cbFormation), 0, 10);
    settingsGrid->addWidget(cbFormation, 0, 11);
    connect(cbFormation, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(buildConfigChanged(const QString&)));

    layout->addRow(settingsGrid);
    QFrame* hr = new QFrame(this);
    hr->setFrameStyle(QFrame::Sunken | QFrame::HLine);
    layout->addRow(hr);

    QGridLayout* teamGrid = new QGridLayout();
    generateRobotViews(teamGrid);
    layout->addRow(teamGrid);

    setLayout(layout);
  }
}

TeamView::TeamView(TeamSelector* parent, Team* team)
    : QFrame(parent), teamSelector(parent), team(team), robotViews(), cbColor1(0), cbColor2(0), sbNumber(0), lePort(0),
      cbLocation(0), cbWlanConfig(0), cbBuildConfig(0), cbFormation(0) /*,
                                       handicapSlider(0)*/
{
  init();
}

bool TeamView::eventFilter(QObject* obj, QEvent* event) {
  if (obj == sbNumber && event->type() == QEvent::FocusOut) {
    sbNumber->setValue(team->number);
  }
  return QWidget::eventFilter(obj, event);
}

void TeamView::generateRobotViews(QGridLayout* teamGrid) {
  std::vector<std::vector<Robot*>> robots = team->getPlayersPerNumber();
  size_t max = robots.size();
  bool backup = true;
  teamGrid->addWidget(new QLabel("Select All Robots: "), 0, 0);
  QCheckBox* cbSelectedRobots = new QCheckBox(this);
  teamGrid->addWidget(cbSelectedRobots, 0, 1);
  connect(cbSelectedRobots, SIGNAL(stateChanged(int)), this, SLOT(selectAllChanged(int)));
  for (size_t j = 0; j < 2; ++j) {
    for (size_t i = 0; i < max; ++i) {
      RobotView* rv = new RobotView(teamSelector, robots[i][j], (unsigned short)(i + 1), (unsigned short)j);
      robotViews.push_back(rv);
      teamGrid->addWidget(rv, j > 0 ? 3 : 1, i);
    }
  }
  if (backup) {
    QFrame* hr = new QFrame(this);
    hr->setFrameStyle(QFrame::Sunken | QFrame::HLine);
    teamGrid->addWidget(hr, 2, 0, 1, max);
  }
}

void TeamView::update(size_t index) {
  robotViews[index]->update();
}

void TeamView::teamColorChanged(const QString& teamColor) {
  if (team) {
    team->teamColor = toString(teamColor);
  }
}

void TeamView::keeperColorChanged(const QString& keeperColor) {
  if (team) {
    team->keeperColor = toString(keeperColor);
  }
}

void TeamView::selectAllChanged(int state) {
  for (size_t i = 0; i < team->getPlayersPerNumber().size(); ++i) {
    robotViews[i]->setSelected(state);
    robotViews[i]->update();
  }
}

void TeamView::numberChanged(int number) {
  if (team) {
    if (teamSelector->updateTeamNumber(team->number, (unsigned short)number))
      team->number = (unsigned short)number;
    team->port = (unsigned short)(10000 + number);
  }
}

void TeamView::portChanged(const QString& port) {
  if (team)
    team->port = port.toUShort();
}

void TeamView::setPort(const int port) {
  if (team)
    team->port = (unsigned short)port;
}

void TeamView::locationChanged(const QString& location) {
  if (team)
    team->location = toString(location);
}

void TeamView::wlanConfigChanged(const QString& config) {
  if (team)
    team->wlanConfig = toString(config);
}

void TeamView::buildConfigChanged(const QString& build) {
  if (team)
    team->buildConfig = toString(build);
}
