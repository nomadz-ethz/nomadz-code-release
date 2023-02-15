#include "cmds/PingCmd.h"
#include "cmdlib/Context.h"
#include "cmdlib/Commands.h"
#include "cmdlib/ProcessRunner.h"
#include "tools/StringTools.h"
#include "Session.h"
#include <cstdlib>
#include <QStringList>

#ifdef WIN32
static QStringList getParams(const std::string& ip) {
  QStringList args;
  args << QString("-n") << QString("1") << QString("-w") << QString("2000") << fromString(ip);
  return args;
}

bool PingCmd::PingTask::isReachable(ProcessRunner& r) {
  // some times ping tells me that the host wasn't found but returns an good exit code
  return r.getProcess()->exitCode() == 0 && r.getOutput().contains("TTL=");
}
#else
static QStringList getParams(const std::string& ip) {
  QStringList args;
#ifdef MACOSX
  args << QString("-c1") << QString("-t2") << fromString(ip);
#else
  args << QString("-c1") << QString("-w2") << fromString(ip);
#endif
  return args;
}

bool PingCmd::PingTask::isReachable(ProcessRunner& r) {
  return r.getProcess()->exitCode() == 0;
}
#endif

PingCmd PingCmd::thePingCmd;

PingCmd::PingCmd() : RobotCommand(false) {
  Commands::getInstance().addCommand(this);
}

std::string PingCmd::getName() const {
  return "ping";
}

std::string PingCmd::getDescription() const {
  return "Pings selected robots.";
}

Task* PingCmd::perRobotExecution(Context& context, Robot& robot) {
  return new PingTask(context, &robot);
}

PingCmd::PingTask::PingTask(Context& context, Robot* robot) : RobotTask(context, robot), firstTry(true) {}

bool PingCmd::PingTask::execute() {

  QString command("ping");
  QStringList args;
  std::string ip = robot->getBestIP();
  args = getParams(ip);

  ProcessRunner r(command, args);
  r.run();

  if (!isReachable(r)) {
    if (firstTry) // try other IP
    {
      std::string ip_str = "";
      if (ip.length() > 0)
        ip_str = " IP " + ip;
      context().printLine("Robot \"" + robot->name + "\"" + ip_str + " is unreachable. Trying other IP...");
      firstTry = false;
      Session::getInstance().refreshRobots();
      return execute();
    }
    context().errorLine("Robot \"" + robot->name + "\" is unreachable");
    firstTry = true;
    return false;
  }
  context().printLine("Robot \"" + robot->name + "\" is available");
  firstTry = true;
  return true;
}
