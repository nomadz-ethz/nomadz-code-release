#pragma once

#include <map>
#include <string>
#include <vector>
#include <QObject>

class PingAgent;
class RemoteRobotAgent;
class StatusAgent;
class TeamCommAgent;
class Robot;
class Team;
class IConsole;
class CommandContext;
class MainWindow;

enum ENetwork {
  WLAN = 0,
  LAN,
  ENetworkSize,
  NONE // This is invalid (do not create an entry in the maps for it)
};

enum LogLevel { ALL = 0, TRACE = 0, WARN = 1, CRITICAL = 2, FATAL = 3, OFF = 4 };

class Session : public QObject {
  Q_OBJECT

  friend class Initializer;

  IConsole* console;
  LogLevel logLevel;

  PingAgent* pingAgent;
  StatusAgent* statusAgent;
  std::vector<TeamCommAgent*> teamCommAgents;
  RemoteRobotAgent* remoteRobotAgent;
  MainWindow* mainWindow;

  Session();
  ~Session();

  ENetwork getBestNetwork(const Robot* robot);

public:
  std::map<std::string, Robot*> robotsByName;

  static Session& getInstance();

  MainWindow* getMainWindow() const { return mainWindow; }
  void setMainWindow(MainWindow* mainWindow) { this->mainWindow = mainWindow; }

  /* redo pings to refresh the networks & wait for first robot to reply or 1sec timeout */
  void refreshRobots();

  void registerConsole(IConsole* console);
  IConsole* getConsole();

  void log(LogLevel logLevel, const std::string& message);

  std::string getBestIP(const Robot* robot);
  bool isReachable(const Robot* robot);

  void registerPingListener(QObject* qObject);
  void removePingListener(QObject* qObject);

  void registerStatusListener(QObject* qObject, Robot* robot);
  void removeStatusListener(QObject* qObject, Robot* robot);

  std::vector<std::string> sendDebugRequest(const Robot* robot, const std::string& command);

  void addTeamCommAgent(Team* team);
  void removeTeamCommAgent(Team* team);

  /* save per-robot configuration for all robots */
  void saveRobotsConfiguration();

signals:
  void robotsChanged();
};
