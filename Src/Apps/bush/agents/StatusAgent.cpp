#include "agents/StatusAgent.h"
#include "agents/PingAgent.h"
#include "Session.h"
#include "tools/StringTools.h"
#include "tools/ShellTools.h"
#include "models/Robot.h"

#include "Core/System/Time.h"

#define UPDATE_TIME 5000

StatusAgent::StatusAgent(PingAgent* pingAgent) : pingAgent(pingAgent) {}

StatusAgent::~StatusAgent() {
  for (auto it = processes.cbegin(), end = processes.cend(); it != end; ++it)
    if (it->second)
      delete it->second;
}

void StatusAgent::initialize(std::map<std::string, Robot*>& robotsByName) {
  Session::getInstance().registerPingListener(this);
  for (auto it = robotsByName.cbegin(), end = robotsByName.cend(); it != end; ++it) {
    power[it->first] = Power();
    timeOfLastUpdate[it->first] = -UPDATE_TIME;
    timeOfLastStatusUpdate[it->first] = -UPDATE_TIME;
    processes[it->first] = new QProcess(this);
    connect(processes[it->first], SIGNAL(readyReadStandardOutput()), this, SLOT(statusReadable()));
  }
  emit powerChanged(&this->power);
}

void StatusAgent::setPings(ENetwork, std::map<std::string, double>*) {
  const unsigned currentTime = Time::getRealSystemTime();

  for (auto it = Session::getInstance().robotsByName.cbegin(), end = Session::getInstance().robotsByName.cend(); it != end;
       ++it) {
    if (currentTime - timeOfLastUpdate[it->first] > UPDATE_TIME && pingAgent->getBestNetwork(it->second) != ENetwork::NONE &&
        processes[it->first]->state() == QProcess::NotRunning) {
      const std::string ip = pingAgent->getBestNetwork(it->second) == ENetwork::LAN ? it->second->lan : it->second->wlan;
      const std::string cmd = "( echo -n " + it->second->name + "; cat /var/volatile/tmp/nomadz_status.txt; ) | tr -d '\\n'";
      processes[it->first]->start(fromString(remoteCommandForQProcess(cmd, ip)));
      timeOfLastUpdate[it->first] = currentTime;
    }
    if (currentTime - pingAgent->getLastConnectionTime(it->second) > DISCONNECTED_TIME) {
      reset(it->second);
    }
  }
}

void StatusAgent::statusReadable() {
  QProcess* process = dynamic_cast<QProcess*>(sender());

  const QByteArray data = process->readAllStandardOutput();
  const QString output(data);

  const std::string data_s = toString(output);

  const QStringList s = output.split(' ');
  if (s.size() < 3)
    return;

  const std::string name = toString(s[0]);

  const int statusUpdateTimestamp = s[2].toInt();

  // stale status update
  if(timeOfLastStatusUpdate[name] == statusUpdateTimestamp) {
    power[name].value = 0;
  } else {
    timeOfLastStatusUpdate[name] = statusUpdateTimestamp;
    power[name].value = static_cast<unsigned char>(s[1].toFloat() * 100.f);
  }
 
  emit powerChanged(&this->power);
}

void StatusAgent::reset(Robot* robot) {
  if (robot) {
    power[robot->name].value = 101; // Invalid Value
    timeOfLastUpdate[robot->name] = -UPDATE_TIME;
    timeOfLastStatusUpdate[robot->name] = -UPDATE_TIME;

    emit powerChanged(&this->power);
  }
}
