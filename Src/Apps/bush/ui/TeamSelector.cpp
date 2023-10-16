#include <QFrame>
#include <QGridLayout>
#include <QAction>
#include <QPushButton>
#include <QInputDialog>
#include <QEvent>

#include "models/Team.h"
#include "models/Robot.h"
#include "tools/StringTools.h"
#include "ui/TeamSelector.h"
#include "ui/TeamView.h"
#include "ui/RobotView.h"

TeamSelector::TeamSelector() : teams(), teamsMap(), teamViews() {
  static const size_t NUM_PLAYERS = 14;
  selectActions.reserve(NUM_PLAYERS);
  for (size_t i = 0; i < NUM_PLAYERS; ++i) {
    QAction* a = new QAction(this);
    a->setShortcut(QKeySequence(Qt::Key_F1 + i));
    addAction(a);
    connect(a, SIGNAL(triggered()), this, SLOT(selectPlayer()));
    selectActions.push_back(a);
  }

  QAction* aNext = new QAction(this);
  aNext->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageDown));
  addAction(aNext);
  connect(aNext, SIGNAL(triggered()), this, SLOT(selectNext()));

  QAction* aPrev = new QAction(this);
  aPrev->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageUp));
  addAction(aPrev);
  connect(aPrev, SIGNAL(triggered()), this, SLOT(selectPrev()));

  QWidget* w = new QWidget(this);
  QPushButton* addTeamButton = new QPushButton("&Add Team", w);
  addTeamButton->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
  connect(addTeamButton, SIGNAL(clicked(bool)), this, SLOT(addTeam()));
  saveConfigButton = new QPushButton("&Save Config", w);

  QHBoxLayout* layout = new QHBoxLayout;
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(addTeamButton);
  layout->addWidget(saveConfigButton);
  w->setLayout(layout);
  setCornerWidget(w);

  this->tabBar()->installEventFilter(this);
  setTabsClosable(true);
  connect(this->tabBar(), SIGNAL(tabCloseRequested(int)), this, SLOT(tabCloseRequest(int)));
}
bool TeamSelector::eventFilter(QObject* obj, QEvent* event) {
  if (obj == tabBar() && event->type() == QEvent::MouseButtonDblClick) {
    uiRenameTeam(getSelectedTeam()->number);
  }
  return QWidget::eventFilter(obj, event);
}

bool TeamSelector::updateTeamNumber(unsigned short oldNumber, unsigned short newNumber) {
  // disallow change if new number already exists
  if (teamsMap.find(newNumber) != teamsMap.end())
    return false;
  // update maps
  std::map<unsigned short, Team*>::iterator teamIter = teamsMap.find(oldNumber);
  if (teamIter != teamsMap.end()) {
    Team* value = teamIter->second;
    teamsMap.erase(teamIter);
    teamsMap[newNumber] = value;
  }
  std::map<unsigned short, TeamView*>::iterator viewIter = teamViews.find(oldNumber);
  if (viewIter != teamViews.end()) {
    TeamView* value = viewIter->second;
    teamViews.erase(viewIter);
    teamViews[newNumber] = value;
  }
  return true;
}

int TeamSelector::addTeam(Team* team) {
  // FIXME: this fails if there is a team with the same port
  Session::getInstance().addTeamCommAgent(team);

  int teamIdx = getIndexFromTeamNumber(team->number);
  teams.push_back(team);
  teamsMap[team->number] = team;
  TeamView* teamPage = new TeamView(this, team);
  int index = -1;
  if (teamIdx != -1) {
    index = teamIdx;
    QWidget* oldPage = widget(index);
    removeTab(index);
    oldPage->deleteLater();
    insertTab(index, teamPage, fromString("Team: " + team->name));
  } else {
    index = addTab(teamPage, fromString("Team: " + team->name));
  }
  teamViews[team->number] = teamPage;
  return index;
}

void TeamSelector::addTeam() {
  Team* selected = getSelectedTeam();
  Team* newTeam = selected ? new Team(*selected) : new Team();
  newTeam->number = getFreeTeamNumber(newTeam->number);
  addTeam(newTeam);
  uiRenameTeam(newTeam->number);
}

void TeamSelector::removeTeam(Team* team) {
  int pageIndex = getIndexFromTeamNumber(team->number);
  std::map<unsigned short, TeamView*>::iterator viewIndexIter = teamViews.find(team->number);
  if (viewIndexIter != teamViews.end()) {
    viewIndexIter->second->deleteLater();
    teamViews.erase(viewIndexIter);
  }
  Session::getInstance().removeTeamCommAgent(team);
  teamsMap.erase(team->number);
  for (size_t i = 0; i < teams.size(); ++i) {
    Team* t = teams[i];
    if (t->name == team->name) {
      teams.erase(teams.begin() + i);
      delete t;
    }
  }
  if (pageIndex != -1)
    removeTab(pageIndex);
  // do not use team from here since it can be invalid
}

Team* TeamSelector::getSelectedTeam() const {
  if (!teams.size())
    return 0;
  int i = currentIndex();
  return teams[i >= 0 ? i : 0];
}

std::vector<Robot*> TeamSelector::getSelectedRobots() const {
  Team* selectedTeam = getSelectedTeam();
  if (selectedTeam)
    return selectedTeam->getSelectedPlayers();
  else {
    Session::getInstance().log(CRITICAL, "TeamSelector: No team selected.");
    return std::vector<Robot*>();
  }
}

void TeamSelector::loadTeams(const QString& filename, bool overwrite) {
  if (overwrite) {
    size_t teamCount = teams.size();
    for (size_t i = 0; i < teamCount; ++i)
      removeTeam(teams[i]);
  }

  loadedTeamsFilename = toString(filename);
  std::vector<Team> loadedTeams = Team::getTeams(loadedTeamsFilename);
  for (size_t i = 0; i < loadedTeams.size(); ++i)
    addTeam(new Team(loadedTeams[i]));
}

void TeamSelector::saveTeams(const QString& filename) {
  std::vector<Team> _teams;
  _teams.reserve(teams.size());
  for (size_t i = 0; i < teams.size(); ++i)
    _teams.push_back(*teams[i]);
  ConfigMap cm;
  Team::writeTeams(cm, _teams);
  std::string fn = toString(filename);
  if (fn.length() == 0)
    fn = loadedTeamsFilename;
  cm.write(&fn);
}

void TeamSelector::selectPlayer() {
  QObject* s = sender();
  if (!s)
    return;
  int number = -1;
  for (size_t i = 0; i < selectActions.size(); ++i)
    if (selectActions[i] == s)
      number = i;
  if (number >= 0) {
    Team* t = getSelectedTeam();
    t->setSelectPlayer(number, !t->isPlayerSelected(number));
    teamViews[t->number]->update(number);
  }
}

void TeamSelector::selectNext() {
  if (currentIndex() + 1 < count())
    setCurrentIndex(currentIndex() + 1);
}

void TeamSelector::selectPrev() {
  if (currentIndex() > 0)
    setCurrentIndex(currentIndex() - 1);
}

unsigned short TeamSelector::getFreeTeamNumber(unsigned short initial) {
  while (teamViews.find(++initial) != teamViews.end())
    ;
  return initial;
}

void TeamSelector::uiRenameTeam(unsigned short teamNumber) {
  bool ok;
  Team* team = teamsMap[teamNumber];
  QString text =
    QInputDialog::getText(this, "Team Name", "New Team Name:", QLineEdit::Normal, QString(team->name.c_str()), &ok);
  if (ok && !text.isEmpty()) {
    team->name = toString(text);
    setTabText(getIndexFromTeamNumber(teamNumber), fromString("Team: " + team->name));
  }
}

void TeamSelector::tabCloseRequest(int index) {
  if (teams.size() <= 1 || index < 0)
    return;
  removeTeam(teams[index]);
}

int TeamSelector::getIndexFromTeamNumber(unsigned short teamNumber) {
  for (int i = 0; i < (int)teams.size(); ++i) {
    if (teams[i]->number == teamNumber)
      return i;
  }
  return -1;
}
