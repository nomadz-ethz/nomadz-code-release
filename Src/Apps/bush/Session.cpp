#include "Session.h"
#include "models/Robot.h"
#include "models/Team.h"
#include "agents/PingAgent.h"
#include "agents/StatusAgent.h"
#include "agents/TeamCommAgent.h"
#include "agents/RemoteRobotAgent.h"
#include "cmdlib/IConsole.h"
#include <QTimer>
#include <QEventLoop>
#include <iostream>

Session::Session()
    : console(0), logLevel(ALL), pingAgent(0), statusAgent(0), teamCommAgents(), remoteRobotAgent(0), mainWindow(0),
      robotsByName() {}

Session::~Session() {}

std::string Session::getBestIP(const Robot* robot) {
  std::string ip;
  ENetwork best = getBestNetwork(robot);
  if (best == WLAN)
    ip = robot->wlan;
  else if (best == LAN)
    ip = robot->lan;

  return ip;
}

Session& Session::getInstance() {
  static Session theInstance;
  return theInstance;
}

void Session::registerConsole(IConsole* console) {
  this->console = console;
  log(TRACE, "Session: Console registered.");
}

IConsole* Session::getConsole() {
  return console;
}

void Session::log(LogLevel logLevel, const std::string& error) {
  if (logLevel >= this->logLevel) {
    if (console)
      console->errorLine(error);
    else
      std::cerr << error << std::endl;
  }
}

bool Session::isReachable(const Robot* robot) {
  return getBestNetwork(robot) != NONE;
}

ENetwork Session::getBestNetwork(const Robot* robot) {
  return pingAgent->getBestNetwork(robot);
}

void Session::registerPingListener(QObject* qObject) {
  if (pingAgent) {
    log(TRACE, "Session: Registered ping listener.");
    QObject::connect(pingAgent,
                     SIGNAL(pingChanged(ENetwork, std::map<std::string, double>*)),
                     qObject,
                     SLOT(setPings(ENetwork, std::map<std::string, double>*)));
  } else {
    log(WARN, "Session: Could not register ping listener. No pingAgent initialized.");
  }
}

void Session::removePingListener(QObject* qObject) {
  log(TRACE, "Session: Removed ping listener.");
  QObject::disconnect(pingAgent,
                      SIGNAL(pingChanged(ENetwork, std::map<std::string, double>*)),
                      qObject,
                      SLOT(setPings(ENetwork, std::map<std::string, double>*)));
}

void Session::registerStatusListener(QObject* qObject, Robot* robot) {
  if (statusAgent) {
    log(TRACE, "Session: Registered status listener.");
    QObject::connect(statusAgent,
                     SIGNAL(powerChanged(std::map<std::string, Power>*)),
                     qObject,
                     SLOT(setPower(std::map<std::string, Power>*)));
    statusAgent->reset(robot);
  } else {
    log(WARN, "Session: Could not register status listener. No statusAgent initialized.");
  }
}

void Session::removeStatusListener(QObject* qObject, Robot* robot)
{
  log(TRACE, "Session: Removed status listener.");
  QObject::disconnect(statusAgent, SIGNAL(powerChanged(std::map<std::string, Power>*)),
                      qObject, SLOT(setPower(std::map<std::string, Power>*)));
  statusAgent->reset(robot);
}

std::vector<std::string> Session::sendDebugRequest(const Robot* robot, const std::string& command) {
  return remoteRobotAgent->sendDebugRequestToRobot(robot, command);
}

void Session::addTeamCommAgent(Team* team) {
  TeamCommAgent* tca = new TeamCommAgent(team->port);
  teamCommAgents.push_back(tca);
  tca->start(tca, &TeamCommAgent::run);
}

void Session::removeTeamCommAgent(Team* team) {
  for (size_t i = 0; i < teamCommAgents.size(); ++i) {
    TeamCommAgent* tca = teamCommAgents[i];
    if (tca->getPort() == team->port) {
      teamCommAgents.erase(teamCommAgents.begin() + i);
      tca->deleteLater();
    }
  }
}

void Session::refreshRobots() {
  emit robotsChanged();

  // wait for the first ping reply or a timeout (this reply could be from any robot...)
  QEventLoop loop;
  QTimer timer;
  connect(pingAgent, SIGNAL(pingChanged(ENetwork, std::map<std::string, double>*)), &loop, SLOT(quit()));
  connect(&timer, SIGNAL(timeout()), &loop, SLOT(quit()));
  timer.start(1000);
  loop.exec(); // blocks untill either theSignalToWaitFor or timeout was fired
}

void Session::saveRobotsConfiguration() {
  for (std::map<std::string, Robot*>::iterator robot = robotsByName.begin(); robot != robotsByName.end(); ++robot) {
    robot->second->writeBehaviorConfig();
  }
}
