#include "cmds/CompileCmd.h"
#include "cmdlib/Context.h"
#include "tools/Filesystem.h"
#include "tools/StringTools.h"
#include "models/Team.h"
#include "Core/System/File.h"
#include "tools/Platform.h"
#include "models/Robot.h"

CompileCmd CompileCmd::theCompileCmd;

CompileCmd::CompileCmd() {
  Commands::getInstance().addCommand(this);
}

std::string CompileCmd::getName() const {
  return "compile";
}

std::string CompileCmd::getDescription() const {
  return "[ <config> [ <project> ] ]\nCompiles a project with a specified build configuration. [Default: Develop and Nao]";
}

std::vector<std::string> CompileCmd::complete(const std::string& cmdLine) const {
  std::vector<std::string> commandWithArgs = split(cmdLine);

  if (commandWithArgs.size() == 1)
    return getBuildConfigs();
  else if (commandWithArgs.size() == 2 && *--cmdLine.end() != ' ')
    return getBuildConfigs(commandWithArgs[1]);
  else if (commandWithArgs.size() == 2)
    return Filesystem::getProjects("");
  else
    return Filesystem::getProjects(commandWithArgs[2]);
}

CompileCmd::CompileTask::CompileTask(Context& context,
                                     const std::string& label,
                                     const QString& command,
                                     const QStringList& args)
    : Task(context), r(context, command, args), label(label) {}

bool CompileCmd::CompileTask::execute() {
  r.run();
  if (context().isCanceled()) {
    context().cleanupFinished();
    return true;
  }
  bool status = true;
  if (r.error()) {
    context().errorLine("Failed to compile.");
    status = false;
  }
  return status;
}

void CompileCmd::CompileTask::cancel() {
  r.stop();
}

void CompileCmd::CompileTask::setContext(Context* context) {
  r.setContext(*context);
  Task::setContext(context);
}

std::string CompileCmd::CompileTask::getLabel() {
  return label;
}

#ifdef CMAKE_BUILD
#define LABEL "cmake"
#else
#ifdef LINUX
#define LABEL "make"
#else
#ifdef MACOSX
#define LABEL "xcodebuild"
#else
#ifdef WIN32
#define LABEL "vcproj"
#endif
#endif
#endif
#endif

bool CompileCmd::execute(Context& context, const std::vector<std::string>& params) {
  QString command = getCommand();
  QStringList args;

  if (params.size() > 2) {
    context.errorLine("Too many parameters specified.");
    return false;
  } else if (params.empty()) {
    Team* team = context.getSelectedTeam();
    if (team && team->buildConfig.length() > 0)
      args = getParams(fromString(team->buildConfig), "Nao", context);
    else
      args = getParams("Develop", "Nao", context);
  } else if (params.size() == 1)
    args = getParams(fromString(params[0]), "Nao", context);
  else
    args = getParams(fromString(params[0]), fromString(params[1]), context);

  context.executeDetached(new CompileTask(context, LABEL, command, args));
  return context.waitForChildren();
}

#ifdef BUILD_CMAKE
QString CompileCmd::getCommand() {
  return fromString("bash"); // hack for those who use 64bit systems and bash functions to set right CFLAGS
}

QStringList CompileCmd::getParams(const QString& config, const QString& project, Context& context) {
  QStringList args;
  args << "-c";

  bool v5 = false;
  bool v6 = false;
  for (auto& robot : context.getSelectedRobots()) {
    if (robot->version < 6)
      v5 = true;
    else
      v6 = true;
  }

  QString total_mk;
  QString base_mk;
  if (config == "Release") {
    base_mk += "-DCMAKE_BUILD_TYPE=Release ";
  } else if (config == "Develop") {
    base_mk += "-DCMAKE_BUILD_TYPE=RelWithDebInfo ";
  } else if (config == "Debug") {
    base_mk += "-DCMAKE_BUILD_TYPE=Debug ";
  }
  base_mk += fromString(std::string(File::getBHDir()));

  if (v6) {
    QString v6_dir;
#ifdef TARGET_64BIT
    v6_dir = fromString(std::string(File::getBHDir())) + "/Build/CMake/V6-64";
#else
    v6_dir = fromString(std::string(File::getBHDir())) + "/Build/CMake/V6-32";
#endif
    QString mke = "mkdir -p " + v6_dir + " && cd " + v6_dir + " && cmake ";
    mke += base_mk;

#ifdef TARGET_64BIT
    mke += " -DNOMADZ_TARGET=v6-64";
#else
    mke += " -DNOMADZ_TARGET=v6-32";
#endif
    mke += " -DYOCTO_SDK=" YOCTO_SDK;

#ifdef ENABLE_TFLITE
    mke += " -DENABLE_TFLITE=ON";
#endif
#ifdef ENABLE_ROS_TARGET
    mke += " -DENABLE_ROS=ON";
#endif

    mke += " && cmake --build " + v6_dir + " -- -j$(nproc)";
    total_mk += mke;
  }
  if (v5) {
    if (v6)
      total_mk += " && ";

    QString v5_dir = fromString(std::string(File::getBHDir())) + "/Build/CMake/V5 ";
    QString mke = "mkdir -p " + v5_dir + " && cd " + v5_dir + " && cmake ";
    mke += base_mk;
    mke += " -DNOMADZ_TARGET=v5 ";
    mke += " && cmake --build " + v5_dir + " -- -j$(nproc)";
    total_mk += mke;
  }

  args << total_mk;

  return args;
}
#elif defined(BUILD_COLCON)
QString CompileCmd::getCommand() {
  return fromString("env"); // use env to start a fresh shell
}

QStringList CompileCmd::getParams(const QString& config, const QString& project, Context& context) {
  QStringList args;

  args << "-S -i bash";

  for (auto& robot : context.getSelectedRobots()) {
    if (robot->version < 6) {
      context.errorLine("Versions before v6 not supported");
      return args;
    }
  }

  const char* ros_setup_script_ptr = getenv("NOMADZ_ROS_SETUP_SCRIPT");
  if(ros_setup_script_ptr == nullptr) {
      context.errorLine("Set NOMADZ_ROS_SETUP_SCRIPT env variable to a script to source to initialize the ROS2 environment");
      return args;
  }
  QString ros_setup_script = ros_setup_script_ptr;

  const char* yocto_ptr = getenv("NOMADZ_YOCTO_SDK");
  if(yocto_ptr == nullptr) {
      context.errorLine("Set NOMADZ_YOCTO_SDK env variable to the absolute path pointing to a Nao Yocto-SDK.");
      return args;
  }
  QString yocto_sdk = yocto_ptr;

  const char* nomadz_ng_package_dir_ptr = getenv("NOMADZ_NG_PACKAGE_DIR");
  if(nomadz_ng_package_dir_ptr == nullptr) {
      context.errorLine("Set NOMADZ_NG_PACKAGE_DIR env variable to the absolute path of the nomadz-ng repo.");
      return args;
  }
  QString nomadz_ng_package_dir = nomadz_ng_package_dir_ptr;

  args << "-cl";

  QString total_mk;
  QString cmake_args;
  QString dir = fromString(COLCON_ROBOT_WORKSPACE_DIR);
  if (config == "Release") {
    cmake_args += "-DCMAKE_BUILD_TYPE=Release ";
  } else if (config == "Develop") {
    cmake_args += "-DCMAKE_BUILD_TYPE=RelWithDebInfo ";
  } else if (config == "Debug") {
    cmake_args += "-DCMAKE_BUILD_TYPE=Debug ";
  }

#ifdef ENABLE_TFLITE
  cmake_args += "-DENABLE_TFLITE=ON ";
#endif

  cmake_args += "-DCMAKE_INSTALL_MESSAGE=NEVER ";
  cmake_args += "--no-warn-unused-cli ";

  total_mk += "mkdir -p " + dir + " && cd " + dir + " && mkdir -p src";
  total_mk += " && ln -sfT " + nomadz_ng_package_dir + " src/nomadz-ng";
  total_mk += " && ln -sfT " COLCON_NOMADZ_LEGACY_PACKAGE_DIR " src/nomadz-legacy";
  total_mk += " && source " + ros_setup_script;
  total_mk += " && NOMADZ_YOCTO_SDK=" + yocto_sdk + " colcon build --merge-install --event-handlers console_direct+";
  total_mk +=
    " --cmake-args -DCMAKE_TOOLCHAIN_FILE=" + nomadz_ng_package_dir + "/cmake/v6-64.toolchain.cmake " + cmake_args;

  args << total_mk;

  return args;
}
#else
#ifdef WIN32
QString CompileCmd::getCommand() {
  return QString(getenv(ENV_COMNTOOLS_11)) + "..\\IDE\\devenv.com";
}

QStringList CompileCmd::getParams(const QString& config, const QString& project, Context& context) {
  QStringList args;
  const QString makeDir(fromString(makeDirectory()));
  args << fromString(std::string(File::getBHDir())).replace("/", "\\") + "\\Make\\" + makeDir + "\\B-Human.sln";
  args << QString("/Build") << config << QString("/Project") << project;
  return args;
}
#elif defined(MACOSX)
QString CompileCmd::getCommand() {
  return fromString(std::string(File::getBHDir())) + "/Make/MacOS/compileFromBush";
}

QStringList CompileCmd::getParams(const QString& config, const QString& project, Context& context) {
  QStringList args;
  args << project << config;
  return args;
}
#else
QString CompileCmd::getCommand() {
  return fromString("bash"); // hack for those who use 64bit systems and bash functions to set right CFLAGS
}

QStringList CompileCmd::getParams(const QString& config, const QString& project, Context& context) {
  QStringList args;
  args << "-c";

  QString mke = "make -C " + fromString(std::string(File::getBHDir()) + "/Make/Linux");
  mke += fromString(" CONFIG=") + config + " " + project;

  args << mke;

  return args;
}
#endif // WIN32
#endif
