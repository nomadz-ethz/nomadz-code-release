#include "cmds/DebugRequestCmd.h"
#include "cmdlib/Context.h"
#include "tools/StringTools.h"
#include "tools/ShellTools.h"
#include "tools/Platform.h"
#include "Session.h"
#include "Core/System/File.h"
#include <cstdlib>

DebugRequestCmd DebugRequestCmd::theDebugRequestCmd;

DebugRequestCmd::DebugRequestCmd() {
  Commands::getInstance().addCommand(this);
}

std::string DebugRequestCmd::getName() const {
  return "dr";
}

std::string DebugRequestCmd::getDescription() const {
  return "Sends debug requests.";
}

bool DebugRequestCmd::preExecution(Context& context, const std::vector<std::string>& params) {
  if (params.size() >= 1)
    request = "dr " + params[0];
  if (params.size() == 2)
    request += " " + params[1];
  else if (params.size() > 2) {
    context.errorLine("Too many parameters.");
    return false;
  }

  return true;
}

Task* DebugRequestCmd::perRobotExecution(Context& context, Robot& robot) {
  std::vector<std::string> answer = Session::getInstance().sendDebugRequest(&robot, request);
  if (answer.empty()) {
    context.errorLine(robot.name + ": Could not send debug request.");
  } else {
    context.printLine(robot.name + ": ");
    for (size_t i = 0; i < answer.size(); i++)
      context.printLine("\t" + answer[i]);
  }

  return NULL;
}
