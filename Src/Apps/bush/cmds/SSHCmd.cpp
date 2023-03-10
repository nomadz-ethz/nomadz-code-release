#include "cmds/SSHCmd.h"
#include "cmdlib/Context.h"
#include "cmdlib/Commands.h"
#include "cmdlib/ProcessRunner.h"
#include "tools/ShellTools.h"
#include "tools/StringTools.h"
#include "Session.h"
#include <cstdlib>

SSHCmd SSHCmd::theSSHCmd;

SSHCmd::SSHCmd() {
  Commands::getInstance().addCommand(this);
}

std::string SSHCmd::getName() const {
  return "ssh";
}

std::string SSHCmd::getDescription() const {
  return "executes an command via ssh or opens a ssh session";
}

bool SSHCmd::preExecution(Context& context, const std::vector<std::string>& params) {
  command = "";
  for (std::vector<std::string>::const_iterator param = params.begin(); param != params.end(); ++param)
    command += " " + *param;

  return true;
}

Task* SSHCmd::perRobotExecution(Context& context, Robot& robot) {
  return new SSHTask(context, &robot, command);
}

SSHCmd::SSHTask::SSHTask(Context& context, Robot* robot, const std::string& command)
    : RobotTask(context, robot), command(command) {}

bool SSHCmd::SSHTask::execute() {
  ProcessRunner r(context(), remoteCommandForQProcess(command, robot->getBestIP()));
  r.run();

  if (r.error()) {
    context().errorLine(robot->name + ": ssh command" + command + " failed.");
    return false;
  }
  return true;
}
