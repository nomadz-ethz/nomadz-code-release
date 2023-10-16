#include "Core/System/File.h"
#include <QString>
#include <QStringList>
#include "agents/PingAgent.h"
#include "cmdlib/Context.h"
#include "cmdlib/Commands.h"
#include "cmdlib/ProcessRunner.h"
#include "cmds/DeployCmd.h"
#include "tools/StringTools.h"
#include "tools/Platform.h"
#include "models/Team.h"

DeployCmd DeployCmd::theDeployCmd;

DeployCmd::DeployCmd() {
  Commands::getInstance().addCommand(this);
}

std::string DeployCmd::getName() const {
  return "deploy";
}

std::string DeployCmd::getDescription() const {
  return "Deploys code and configuration to selected robots. (Uses the copyfiles script)";
}

std::vector<std::string> DeployCmd::complete(const std::string& cmdLine) const {
  std::vector<std::string> commandWithArgs = split(cmdLine);

  if (commandWithArgs.size() == 1)
    return getBuildConfigs();
  else
    return getBuildConfigs(commandWithArgs[1]);
}

bool DeployCmd::preExecution(Context& context, const std::vector<std::string>& params) {
  team = context.getSelectedTeam();
  if (!team) {
    context.errorLine("No team selected!");
    return false;
  }

  buildConfig = fromString(team->buildConfig);
  if (params.size() > 0)
    buildConfig = fromString(params[0]);

  // compile and deploy if compiling was successful
  return context.execute("compile " + toString(buildConfig));
}

Task* DeployCmd::perRobotExecution(Context& context, Robot& robot) {
  /* Since the PingAgent knows the roundtrip time for all robots, maybe we can
   * adjust the timeout of rsync to determine faster if the connection is
   * lost.
   */
  QString command = getCommand();

  QStringList args = QStringList();
#ifdef BUILD_COLCON
  args.push_back(COLCON_ROBOT_WORKSPACE_DIR);
#endif
  args.push_back(QString("-nc"));
  args.push_back(buildConfig);
  args.push_back(fromString(robot.getBestIP()));

  ProcessRunner r(context, command, args);
  r.run();
  if (r.error())
    context.errorLine("Deploy of \"" + robot.name + "\" failed!");
  else
    context.printLine("Success! (" + robot.name + ")");
  return NULL;
}

bool DeployCmd::postExecution(Context& context, const std::vector<std::string>& params) {
  // deploy finished, change the settings now
  context.execute("updateSettings");
  // restart the robots
  return context.execute("restart");
}

#ifdef WIN32
QString DeployCmd::getCommand() {
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/copyfiles.cmd");
}
#else
QString DeployCmd::getCommand() {
#ifdef BUILD_CMAKE
  return fromString(std::string(File::getBHDir()) + "/Install/copyfiles");
#elif defined(BUILD_COLCON)
  return fromString(std::string(File::getBHDir()) + "/Install/copyfiles-ng");
#else
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/copyfiles");
#endif
}
#endif
