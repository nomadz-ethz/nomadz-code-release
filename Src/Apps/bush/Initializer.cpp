#include "Initializer.h"
#include "agents/PingAgent.h"
#include "agents/StatusAgent.h"
#include "agents/TeamCommAgent.h"
#include "agents/RemoteRobotAgent.h"
#include "cmdlib/ProcessRunner.h"
#include "models/Robot.h"
#include "models/Team.h"
#include "tools/Platform.h"
#include "tools/StringTools.h"
#include "ui/Console.h"
#include "bhwrapper/Framework.h"

#include <iostream>
#include <cstdlib>

#include <QApplication>
#include <QDir>
#include "ui/MainWindow.h"

Initializer::Initializer(int& argc, char** argv) : logLevel(WARN), app(0) {
  log(TRACE, "Initializer: Initialization started.");

#ifdef WIN32
  ProcessRunner r("taskkill /F /IM ping.exe");
#else // Linux, MacOS
  ProcessRunner r("ps axco pid,command | grep \" ping$\" | awk '{ print $1; }' | xargs kill | echo X");
#endif
  r.run();
  if (r.error())
    log(WARN, "Initializer: Could not kill any ping processes.");
  else
    log(TRACE, "Initializer: Killed all active ping processes.");

  app = new QApplication(argc, argv);
#ifdef MACOSX
  app->setStyle("macintosh");
#endif
  app->setApplicationName("B-Human User Shell (bush)");
  app->setCursorFlashTime(0);
  Icons::getInstance().init();
  log(TRACE, "Initializer: Created Qt application.");

  log(TRACE, "Initializer: Changing working directory.");
  goToConfigDirectory(argv[0]);
  Session::getInstance();
  log(TRACE, "Initializer: Session instance created.");

  Session::getInstance().logLevel = WARN;
  log(TRACE, "Initializer: Set log level to " + toString(Session::getInstance().logLevel) + ".");

  Session::getInstance().pingAgent = new PingAgent;
  log(TRACE, "Initializer: Ping agent started.");

  Session::getInstance().statusAgent = new StatusAgent(Session::getInstance().pingAgent);
  log(TRACE, "Initializer: Power agent started.");

  // this initializes the Global::Settings which we need to load the robots
  Framework::getInstance("bush.global");
  log(TRACE, "Initializer: dummy Framework initialized.");

  Session::getInstance().remoteRobotAgent = new RemoteRobotAgent;
  log(TRACE, "Initializer: Remote robot agent started.");

  Robot::initRobotsByName(Session::getInstance().robotsByName);
  log(TRACE, "Initializer: Robots loaded.");

  Session::getInstance().pingAgent->connect(&Session::getInstance(), SIGNAL(robotsChanged()), SLOT(robotsChanged()));
  Session::getInstance().pingAgent->initializeProcesses(Session::getInstance().robotsByName);
  log(TRACE, "Initializer: Registered robots at ping agent.");

  Session::getInstance().statusAgent->initialize(Session::getInstance().robotsByName);
  log(TRACE, "Initializer: Registered robots at status agent.");

  mainWindow = new MainWindow;
  mainWindow->show();
  log(TRACE, "Initializer: Created main window.");

  log(TRACE, "Initializer: Finished initialization.");
}

Initializer::~Initializer() {
  log(TRACE, "Initializer: Shutdown started.");

  mainWindow->deleteLater();
  app->deleteLater();
  log(TRACE, "Initializer: Deleted GUI.");

  Session::getInstance().console = 0;
  log(TRACE, "Initializer: Removed console.");
  for (std::vector<TeamCommAgent*>::iterator it = Session::getInstance().teamCommAgents.begin();
       it != Session::getInstance().teamCommAgents.end();
       it++)
    (*it)->announceStop();
  log(TRACE, "Initializer: Announced stop to team communication agents.");

  delete Session::getInstance().pingAgent;
  Session::getInstance().pingAgent = 0;
  log(TRACE, "Initializer: Removed ping agent.");
  
  delete Session::getInstance().statusAgent;
  Session::getInstance().statusAgent = 0;
  log(TRACE, "Initializer: Removed status agent.");
  

  for (std::vector<TeamCommAgent*>::iterator it = Session::getInstance().teamCommAgents.begin();
       it != Session::getInstance().teamCommAgents.end();
       it++)
    (*it)->deleteLater();
  Session::getInstance().teamCommAgents.clear();
  log(TRACE, "Initializer: Removed team comm agents.");

  delete Session::getInstance().remoteRobotAgent;
  Session::getInstance().remoteRobotAgent = 0;
  log(TRACE, "Initializer: Removed remote robot agent.");

  log(TRACE, "Initializer: Finished shutdown.");
}

int Initializer::start() {
  return app->exec();
}

void Initializer::log(LogLevel logLevel, const std::string& message) {
  if (logLevel >= this->logLevel) {
    std::cout << message << std::endl;
  }
}
