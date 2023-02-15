#pragma once

#include <QTabWidget>
#include <QString>
#include <QPushButton>
#include <map>
#include <vector>
#include <string>

class Team;
class Robot;
class QFrame;
class QAction;
class RobotView;
class TeamView;

class TeamSelector : public QTabWidget {
  Q_OBJECT

  std::vector<Team*> teams;
  std::map<unsigned short, Team*> teamsMap;
  std::map<unsigned short, TeamView*> teamViews;
  std::vector<QAction*> selectActions;
  std::string loadedTeamsFilename;
  void generateRobotViews(Team& team, QFrame* teamPage);
  /* get an unused team number */
  unsigned short getFreeTeamNumber(unsigned short initial);

  /* get tab view index from a team number (-1 if none is found) */
  int getIndexFromTeamNumber(unsigned short teamNumber);

public:
  TeamSelector();
  int addTeam(Team* team); // returns tab index of new item
  /** Deletes delivered team.
   * Do not use the pointer after a call of removeTeam if it poited to the same
   * memory as the pointer in teams.
   */
  void removeTeam(Team* team);
  Team* getSelectedTeam() const;
  std::vector<Robot*> getSelectedRobots() const;
  void loadTeams(const QString& filename = "", bool overwrite = true);
  /* for empty file name, the same one will be used as for loadTeams */
  void saveTeams(const QString& filename = "");

  /*
   * prompts the user for a new team name & changes the team name
   * @teamNumber number of the team to be renamed
   */
  void uiRenameTeam(unsigned short teamNumber);

  bool eventFilter(QObject* obj, QEvent* event);

  /*
   * change an existing team number. returns false if it cannot be changed
   * does not change Team::number, but only the other internal representations
   */
  bool updateTeamNumber(unsigned short oldNumber, unsigned short newNumber);

  QPushButton* saveConfigButton;
private slots:
  void selectPlayer();
  void selectNext();
  void selectPrev();
  /*
   * add a new team & prompt for new team name
   */
  void addTeam();

  void tabCloseRequest(int index);
};
