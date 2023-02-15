#pragma once

#include "cmdlib/RobotCommand.h"

class SimCmd : public RobotCommand {
  std::string simulatorExecutable;
  std::string remoteRobotScene;
  std::string connectScript;

  SimCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool preExecution(Context& context, const std::vector<std::string>& params);
  virtual Task* perRobotExecution(Context& context, Robot& robot);
  std::string getSimulatorExecutable(const std::string& buildConfig);

public:
  static SimCmd theSimCmd;
};
