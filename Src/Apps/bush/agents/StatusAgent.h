#pragma once

#include "Session.h"
#include "models/Power.h"
#include <map>
#include <string>
#include <QObject>
#include <QProcess>

class PingAgent;
struct Robot;

class StatusAgent : public QObject
{
  Q_OBJECT

  PingAgent* pingAgent;
  std::map<std::string, Power> power;
  std::map<std::string, int> timeOfLastUpdate;
  std::map<std::string, int> timeOfLastStatusUpdate;
  std::map<std::string, QProcess*> processes;

public:
  StatusAgent(PingAgent* pingAgent);
  ~StatusAgent();
  void initialize(std::map<std::string, Robot*>& robotsByName);

  void reset(Robot* robot);

private slots:
  void setPings(ENetwork, std::map<std::string, double>*);
  void statusReadable();

signals:
  void powerChanged(std::map<std::string, Power>*);
};
