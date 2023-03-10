#include "cmdlib/Context.h"
#include "cmdlib/Commands.h"
#include "cmdlib/ProcessRunner.h"
#include "cmds/ChangeWirelessCmd.h"
#include "models/Team.h"
#include "Session.h"
#include "tools/Filesystem.h"
#include "tools/ShellTools.h"
#include "tools/StringTools.h"

#include <QString>

ChangeWirelessCmd::ChangeWirelessCmd() {
  Commands::getInstance().addCommand(this);
}

ChangeWirelessCmd::ChangeWirelessTask::ChangeWirelessTask(Context& context, Robot* robot, const QString& config)
    : RobotTask(context, robot), config(config) {}

bool ChangeWirelessCmd::ChangeWirelessTask::execute() {
  std::string ip = robot->getBestIP();

  context().printLine(robot->name + ": changing wpa_supplicant config to " + toString(config));
  std::string command = remoteCommand("setprofile " + toString(config), ip);
  ProcessRunner r(context(), fromString(command));
  r = ProcessRunner(context(), fromString(command));
  r.run();
  if (r.error()) {
    context().errorLine(robot->name + ": change wireless failed.");
    return false;
  }
  context().printLine(robot->name + ": change wireless finished.");
  return true;
}

std::string ChangeWirelessCmd::getName() const {
  return "changeWireless";
}

std::string ChangeWirelessCmd::getDescription() const {
  return "changes the active wpa_supplicant wireless configuration of selected robots (obsolete command).";
}

std::vector<std::string> ChangeWirelessCmd::complete(const std::string& cmdLine) const {
  std::vector<std::string> commandWithArgs = split(cmdLine);

  if (commandWithArgs.size() == 1)
    return Filesystem::getWlanConfigs();
  else
    return Filesystem::getWlanConfigs(commandWithArgs[1]);
}

bool ChangeWirelessCmd::preExecution(Context& context, const std::vector<std::string>& params) {
  Team* team = context.getSelectedTeam();
  config = fromString(team->wlanConfig);
  if (!params.empty()) {
    if (params.size() == 1)
      config = fromString(params[0]);
    else {
      context.errorLine("Too many configurations.");
      return false;
    }
  }

  return true;
}

Task* ChangeWirelessCmd::perRobotExecution(Context& context, Robot& robot) {
  return new ChangeWirelessCmd::ChangeWirelessTask(context, &robot, config);
}

ChangeWirelessCmd ChangeWirelessCmd::theChangeWirelessCmd;
