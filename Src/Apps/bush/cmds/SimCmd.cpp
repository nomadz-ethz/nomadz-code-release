#include "cmds/SimCmd.h"
#include "cmdlib/Context.h"
#include "cmdlib/Commands.h"
#include "cmdlib/ProcessRunner.h"
#include "tools/Platform.h"
#include "tools/Sleeper.h"
#include "Session.h"
#include "Core/System/File.h"
#include <cstdlib>

#ifdef LINUX
#include <unistd.h>
#include <limits.h>
#include <libgen.h>
#endif

SimCmd SimCmd::theSimCmd;

SimCmd::SimCmd() {
  Commands::getInstance().addCommand(this);
}

std::string SimCmd::getName() const {
  return "sim";
}

std::string SimCmd::getDescription() const {
  return "Connects to robots via simulator. Requires a Simulator built with Develop configuration.";
}

bool SimCmd::preExecution(Context& context, const std::vector<std::string>& params) {
  const std::string buildConfig = "Develop";
  simulatorExecutable = getSimulatorExecutable(buildConfig);
  remoteRobotScene = File::getBHDir() + linuxToPlatformPath("/Config/Scenes/ScriptRemoteRobot.ros2");
  connectScript = File::getBHDir() + linuxToPlatformPath("/Config/Scenes/scriptconnect.con");

  File simFile(simulatorExecutable, "r");
  if (!simFile.exists()) {
    bool compileStatus = Commands::getInstance().execute(&context, "compile " + buildConfig + " SimRobot");
    if (!compileStatus)
      return false;
  }

  return true;
}

Task* SimCmd::perRobotExecution(Context& context, Robot& robot) {
  Sleeper::msleep(1000); // since the same file is used for every robot
  File* conFile = new File(connectScript, "w");
  std::string command = "sc Remote " + robot.getBestIP();
  conFile->write(command.c_str(), command.size());
  delete conFile;

  ProcessRunner r(context, simulatorExecutable + " " + remoteRobotScene);
  r.run();
  if (r.error())
    context.errorLine("Failed.");

  return 0;
}

std::string SimCmd::getSimulatorExecutable(const std::string& buildConfig) {
#if defined(BUILD_CMAKE) && defined(LINUX)
  char exePath[PATH_MAX], *exeDir;
  ssize_t len = ::readlink("/proc/self/exe", exePath, sizeof(exePath));
  if (len == -1 || len == sizeof(exePath))
    len = 0;
  exePath[len] = '\0';
  exeDir = dirname(exePath);
  std::string simulatorExecutable = std::string(exeDir) + "/SimRobot";
#elif defined(BUILD_COLCON)
  std::string simulatorExecutable = std::string(COLCON_INSTALL_DIR) + "bin/SimRobot";
#else
  std::string simulatorExecutable =
    File::getBHDir() + linuxToPlatformPath("/Build/SimRobot/" + platformDirectory() + "/" + buildConfig + "/SimRobot");
#endif
#ifdef MACOSX
  simulatorExecutable += ".app/Contents/MacOS/SimRobot";
#elif defined WIN32
  simulatorExecutable += ".exe";
#endif

  return simulatorExecutable;
}
