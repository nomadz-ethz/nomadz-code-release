#include "cmdlib/RobotCommand.h"
#include "cmdlib/Context.h"
#include "Session.h"

bool RobotCommand::execute(Context& context, const std::vector<std::string>& params) {

  std::vector<Robot*> selectedRobots = context.getSelectedRobots();
  if (selectedRobots.empty()) {
    context.errorLine("No robots selected.");
    return false;
  }

  if (!preExecution(context, params))
    return false;

  for (std::vector<Robot*>::const_iterator robot = selectedRobots.begin(); robot != selectedRobots.end(); robot++) {
    std::string ip;
    if (checkReachability && !Session::getInstance().isReachable(*robot)) {
      context.errorLine("\"" + (*robot)->name + "\" is not reachable.");
      ;
      status = false;
      continue;
    }
    Task* task = perRobotExecution(context, **robot);
    if (task)
      context.executeDetached(task);
  }
  context.waitForChildren();
  status &= context.getStatus();

  return postExecution(context, params);
}
