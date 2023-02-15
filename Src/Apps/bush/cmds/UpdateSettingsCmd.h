#pragma once

#include "cmdlib/Commands.h"
#include "cmdlib/RobotCommand.h"
#include "models/Team.h"
#include "models/Robot.h"

class UpdateSettingsCmd : public RobotCommand {
  class UpdateSettingsTask : public RobotTask {
    /** We want a copy of the team since it is not good if it is changed during
     * the command.
     */
    const Team team;
    const QString& keyFile;

  public:
    UpdateSettingsTask(Context& context, Robot* robot, const Team& team, const QString& keyFile);
    bool execute();
  };

  Team* team;
  QString keyFile;
  QString location;

  UpdateSettingsCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool preExecution(Context& context, const std::vector<std::string>& params);
  virtual Task* perRobotExecution(Context& context, Robot& robot);

public:
  static UpdateSettingsCmd theUpdateSettingsCmd;
};
