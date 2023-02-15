#pragma once

#include "tools/ConfigMap.h"
#include <string>
#include <vector>

#include "Representations/BehaviorControl/BehaviorConfig.h"

class Session;
class Team;

class Robot {
  static std::map<std::string, Robot*> getRobotsByName();

  bool readBehaviorConfig();

public:
  std::string lan;
  std::string wlan;
  std::string name;
  int version;

  std::string configFileName;
  BehaviorConfig behaviorConfig;

  /* save current behavior config to file */
  void writeBehaviorConfig();

  std::string getSettingsString(const Team& team) const;
  std::string getBestIP() const;

  static std::vector<Robot> getRobots();
  static void initRobotsByName(std::map<std::string, Robot*>& robotsByName);

  friend class Session;
};

CONFIGMAP_STREAM_IN_DELCARE(Robot);
CONFIGMAP_STREAM_OUT_DELCARE(Robot);

ConfigMap& operator<<(ConfigMap& cv, const Robot& robot);
const ConfigMap& operator>>(const ConfigMap& cv, Robot& robot);
