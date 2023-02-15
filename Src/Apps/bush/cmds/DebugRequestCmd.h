#pragma once

#include "cmdlib/Commands.h"
#include "cmdlib/RobotCommand.h"

class DebugRequestCmd;

class DebugRequestCmd : public RobotCommand {
  std::string request;

  DebugRequestCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool preExecution(Context& context, const std::vector<std::string>& params);
  virtual Task* perRobotExecution(Context& context, Robot& robot);

public:
  static DebugRequestCmd theDebugRequestCmd;
};
