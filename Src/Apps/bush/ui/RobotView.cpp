#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
#include <QDrag>
#include <QDropEvent>
#include <QDragEnterEvent>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMimeData>
#include <QMouseEvent>
#include <QPalette>
#include <QProgressBar>
#ifdef __clang__
#pragma clang diagnostic pop
#endif
#include "models/Robot.h"
#include "models/Team.h"
#include "ui/RobotView.h"
#include "ui/TeamSelector.h"
#include "ui/RobotPool.h"
#include "tools/StringTools.h"
#include "Session.h"

#include "Representations/BehaviorControl/BehaviorStatus.h"

void RobotView::init() {
  QFormLayout* layout = new QFormLayout();

  if (playerNumber)
    cPlayerNumber = new QLabel(QString("<font size=5><u><b>") + QString::number(playerNumber) + QString("</b></u></font>"));

  statusWidget = new QWidget(this);
  configWidget = new QWidget(this);
  statusWidget->setMaximumSize(200, 75);
  this->setMaximumWidth(170);
  this->setMaximumHeight(160);
  QGridLayout* statusLayout = new QGridLayout(statusWidget);
  QGridLayout* configLayout = new QGridLayout(configWidget);

  QLabel* pingLabelWLAN = new QLabel("<font size=2><b>Wlan</b></font>", statusWidget);
  pingBarWLAN = new QProgressBar(this);
  pingBarWLAN->setMaximumSize(50, 10);
  pingBarWLAN->setRange(0, 2000);
  setPings(WLAN, 0);
  pingBarWLAN->setTextVisible(false);
  statusLayout->addWidget(pingLabelWLAN, 0, 0, Qt::AlignLeft);
  statusLayout->addWidget(pingBarWLAN, 0, 1);

  QLabel* powerLabel = new QLabel("<font size=2><b>Power</b></font>", statusWidget);
  powerBar = new QProgressBar(this);
  powerBar->setMaximumSize(50, 10);
  powerBar->setRange(0, 100);
  powerBar->setValue(0);
  statusLayout->addWidget(powerLabel, 2, 0, Qt::AlignLeft);
  statusLayout->addWidget(powerBar, 2, 1);

  QLabel* pingLabelLAN = new QLabel("<font size=2><b>Lan</b></font>", statusWidget);
  pingBarLAN = new QProgressBar(this);
  pingBarLAN->setMaximumSize(50, 10);
  pingBarLAN->setRange(0, 2000);
  setPings(LAN, 0);
  pingBarLAN->setTextVisible(false);
  statusLayout->addWidget(pingLabelLAN, 1, 0, Qt::AlignLeft);
  statusLayout->addWidget(pingBarLAN, 1, 1);

  layout->addRow(cPlayerNumber, statusWidget);

  roleCombo = new QComboBox(this);
  for (int i = 0; i < BehaviorStatus::numOfRoles; ++i)
    roleCombo->addItem(BehaviorStatus::getName((BehaviorStatus::Role)i));
  connect(roleCombo, SIGNAL(activated(int)), this, SLOT(roleChanged(int)));
  configLayout->addWidget(roleCombo, 0, 0);
  layout->addRow(configWidget);

  setLayout(layout);
  connect(this, SIGNAL(toggled(bool)), this, SLOT(setSelected(bool)));

  if (robot) {
    Session::getInstance().registerPingListener(this);
    Session::getInstance().registerStatusListener(this, robot);
  }

  update();
  setAcceptDrops(true);
}

void RobotView::update() {
  if (playerNumber)
    cPlayerNumber->setVisible(false);
  statusWidget->setVisible(false);
  configWidget->setVisible(false);
  setCheckable(false);
  if (robot) {
    Robot* r = robot;
    robot = 0;
    setCheckable(playerNumber);
    robot = r;
    if (playerNumber)
      setChecked(teamSelector->getSelectedTeam()->isPlayerSelected(robot));
    std::string ipPostfix = robot->wlan.substr(robot->wlan.length() - 2);
    setTitle(fromString(robot->name + " (." + ipPostfix + ")"));
    roleCombo->setCurrentIndex((int)robot->behaviorConfig.role);
    if (playerNumber) {
      cPlayerNumber->setVisible(true);
      configWidget->setVisible(true);
    }
    statusWidget->setVisible(true);
    configWidget->setEnabled(true);
  } else
    setTitle(fromString("Empty"));
}

RobotView::RobotView(TeamSelector* teamSelector, Robot* robot, unsigned short playerNumber, unsigned short pos)
    : QGroupBox(teamSelector), teamSelector(teamSelector), robot(robot), playerNumber(playerNumber), pos(pos),
      cPlayerNumber(0) {
  init();
}

RobotView::RobotView(TeamSelector* teamSelector, Robot* robot)
    : QGroupBox(teamSelector), teamSelector(teamSelector), robot(robot), playerNumber(0), pos(0), cPlayerNumber(0) {
  init();
}

RobotView::~RobotView() {}

QString RobotView::getRobotName() const {
  if (!robot)
    return "";
  return fromString(robot->name);
}

bool RobotView::isSelected() const {
  if (!robot)
    return false;
  return teamSelector->getSelectedTeam()->isPlayerSelected(robot);
}

void RobotView::setRobot(Robot* robot) {
  if (this->robot) {
    Session::getInstance().removePingListener(this);
    Session::getInstance().removeStatusListener(this, this->robot);
    
  }
  this->robot = robot;
  if (playerNumber) {
    Team* team = teamSelector->getSelectedTeam();
    team->changePlayer(playerNumber, pos, robot);
  }
  if (robot) {
    Session::getInstance().registerPingListener(this);
    Session::getInstance().registerStatusListener(this, robot);
  }
  emit robotChanged();
}

void RobotView::setPings(ENetwork network, std::map<std::string, double>* pings) {
  int value = 2000;
  if (pings)
    value = static_cast<int>((*pings)[robot->name]);
  if (network == LAN)
    pingBarLAN->setValue(value);
  else if (network == WLAN)
    pingBarWLAN->setValue(value);
}

void RobotView::setPower(std::map<std::string, Power>* power) {
  int value = 0;
  if (power && (*power)[robot->name].isValid())
    value = static_cast<int>((*power)[robot->name]);
  powerBar->setValue(value);
}

void RobotView::mouseMoveEvent(QMouseEvent* me) {
  if (!robot)
    return;
  QDrag* d = new QDrag(this);
  QPixmap pm = QPixmap::grabWidget(this, rect());
  d->setPixmap(pm);
  d->setHotSpot(me->pos());
  QMimeData* data = new QMimeData();
  data->setText(fromString(robot->name));
  d->setMimeData(data);
  d->exec(Qt::MoveAction);
  me->accept();
}

void RobotView::dragEnterEvent(QDragEnterEvent* e) {
  if (e->source() && e->source() != this && e->source()->inherits("RobotView"))
    e->acceptProposedAction();
}

void RobotView::dropEvent(QDropEvent* e) {
  e->accept();
  QString robotName = e->mimeData()->text();
  Robot* r = Session::getInstance().robotsByName[toString(robotName)];
  RobotView* source = dynamic_cast<RobotView*>(e->source());
  if (source->playerNumber) {
    bool selected = source->isSelected();
    if (source->robot)
      source->setSelected(false);
    if (robot)
      source->setRobot(robot);
    else
      source->setRobot(0);
    source->setSelected(selected);
    source->update();
    setRobot(r);
    update();
  } else {
    bool selected = isSelected();
    setSelected(false);
    setRobot(r);
    setSelected(selected);
    update();
    source->setRobot(r);
  }
}

void RobotView::setSelected(bool selected) {
  configWidget->setEnabled(true);
  if (robot)
    teamSelector->getSelectedTeam()->setSelectPlayer(robot, selected);
}

void RobotView::roleChanged(int newIndex) {
  if (robot)
    robot->behaviorConfig.role = (BehaviorStatus::Role)newIndex;
}
