#pragma once

#include <QFrame>
#include <vector>

class Team;
class QComboBox;
class QLineEdit;
class QSpinBox;
class QSlider;
class QLabel;
class QGridLayout;
class TeamSelector;
class RobotView;

class TeamView : public QFrame {
  Q_OBJECT

  TeamSelector* teamSelector;
  Team* team;

  std::vector<RobotView*> robotViews;
  QComboBox *cbColor1, *cbColor2;
  QSpinBox* sbNumber;
  QLineEdit* lePort;
  QComboBox* cbLocation;
  QComboBox* cbWlanConfig;
  QComboBox* cbBuildConfig;
  QComboBox* cbFormation;

  void init();

public:
  TeamView(TeamSelector* parent, Team* team);
  void generateRobotViews(QGridLayout* teamGrid);
  void update(size_t index);

  bool eventFilter(QObject* obj, QEvent* event);
private slots:
  void teamColorChanged(const QString& teamColor);
  void keeperColorChanged(const QString& keeperColor);
  void selectAllChanged(int state);
  void numberChanged(int number);
  void portChanged(const QString& port);
  void setPort(const int port);
  void locationChanged(const QString& location);
  void wlanConfigChanged(const QString& config);
  void buildConfigChanged(const QString& build);
};
