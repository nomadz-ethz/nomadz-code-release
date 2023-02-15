#include "cmds/UpdateSettingsCmd.h"
#include "cmdlib/Context.h"
#include "cmdlib/ProcessRunner.h"
#include "models/Robot.h"
#include "Session.h"
#include "tools/StringTools.h"
#include "Core/System/File.h"
#include "tools/Filesystem.h"

UpdateSettingsCmd UpdateSettingsCmd::theUpdateSettingsCmd;

UpdateSettingsCmd::UpdateSettingsCmd() {
  Commands::getInstance().addCommand(this);
}

std::string UpdateSettingsCmd::getName() const {
  return "updateSettings";
}

std::string UpdateSettingsCmd::getDescription() const {
  return "Updates settings on selected robots (this includes only player & team number, color and location).";
}

bool UpdateSettingsCmd::preExecution(Context& context, const std::vector<std::string>& params) {
  keyFile = QString::fromUtf8(Filesystem::getNaoKey().c_str());

  team = context.getSelectedTeam();
  if (!team) {
    context.errorLine("No team selected.");
    return false;
  }
  if (params.size() > 1) {
    context.errorLine("Too many locations");
    return false;
  }
  location = fromString(team->location);
  if (!params.empty())
    location = fromString(params[0]);

  return true;
}

Task* UpdateSettingsCmd::perRobotExecution(Context& context, Robot& robot) {
  return new UpdateSettingsTask(context, &robot, *team, keyFile);
}

UpdateSettingsCmd::UpdateSettingsTask::UpdateSettingsTask(Context& context,
                                                          Robot* robot,
                                                          const Team& team,
                                                          const QString& keyFile)
    : RobotTask(context, robot), team(team), keyFile(keyFile) {}

bool UpdateSettingsCmd::UpdateSettingsTask::execute() {
  QString command = "ssh";
  QStringList args;
  args << "-q"; // This disables die man-in-the-middle warning
  args << "-i";
  args << keyFile;
  args << "-o";
  args << "StrictHostKeyChecking=no";
  args << fromString("nao@" + robot->getBestIP());
  args << "cat"
       << "-"
       << ">"
       << "/home/nao/Config/settings.cfg; sync";

  QByteArray data(robot->getSettingsString(team).c_str());
  RemoteWriteProcessRunner r(context(), command, args, data);
  r.run();

  if (r.error()) {
    context().errorLine("UpdateSettings of \"" + robot->name + "\" failed!");
    return false;
  } else {
    context().printLine("UpdateSettings of \"" + robot->name + "\" successful!");
    return true;
  }
}
