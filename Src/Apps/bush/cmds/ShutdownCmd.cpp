#include "cmds/ShutdownCmd.h"
#include "cmdlib/Context.h"
#include "cmdlib/Commands.h"
#include "cmdlib/ProcessRunner.h"
#include "tools/ShellTools.h"
#include "tools/StringTools.h"
#include <cstdlib>

ShutdownCmd ShutdownCmd::theShutdownCmd;

ShutdownCmd::ShutdownCmd() {
  Commands::getInstance().addCommand(this);
}

std::string ShutdownCmd::getName() const {
  return "shutdown";
}

std::string ShutdownCmd::getDescription() const {
  return "stop nomadz && stop naobridge && shutdown";
}

bool ShutdownCmd::preExecution(Context& context, const std::vector<std::string>& params) {
  if (!params.empty()) {
    context.errorLine("Unrecognized parameters.");
    return false;
  }
  return true;
}

Task* ShutdownCmd::perRobotExecution(Context& context, Robot& robot) {
  return new ShutdownTask(context, &robot);
}

ShutdownCmd::ShutdownTask::ShutdownTask(Context& context, Robot* robot) : RobotTask(context, robot) {}

bool ShutdownCmd::ShutdownTask::execute() {
  std::string ip = robot->getBestIP();

  context().printLine(robot->name + ": Shutting down...");
  std::string command =
    remoteCommand("systemctl stop nomadz && systemctl stop naobridge && sleep 10 && shutdown -h now", ip);
  ProcessRunner r(context(), fromString(command));
  r = ProcessRunner(context(), fromString(command));
  r.run();
  if (r.error()) {
    context().errorLine(robot->name + ": Shutdown failed.");
    return false;
  }
  context().printLine(robot->name + ": Shutdown finished.");
  return true;
}
