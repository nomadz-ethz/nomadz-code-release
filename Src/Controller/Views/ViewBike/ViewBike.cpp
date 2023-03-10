/**
 * @file ViewBike.cpp
 *
 * Implementation of class ViewBike
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:judy@tzi.de">Judith Müller</a>
 */

#include <QHeaderView>
#include <QFileDialog>
#include <QMessageBox>
#include "Controller/RobotConsole.h"
#include "ViewBike.h"
#include "ViewBikeWidget.h"
#include "Core/System/File.h"

ViewBikeHeaderedWidget::ViewBikeHeaderedWidget(ViewBike& viewBike) {
  viewBikeWidget = new ViewBikeWidget(viewBike, parameters, this);
  fileName = QString("noName.kmc");
  setWidget(viewBikeWidget);
  QHeaderView* headerView = getHeaderView();
  headerView->setMinimumSectionSize(30);
  headerView->resizeSection(0, 100);
  headerView->resizeSection(1, 100);
}

void ViewBikeHeaderedWidget::update() {
  viewBikeWidget->update();
}

QMenu* ViewBikeHeaderedWidget::createFileMenu() const {
  QMenu* menu = new QMenu(tr("&BikeFile"));

  QAction *newAct, *saveAct, *saveAsAct, *loadAct;

  newAct = new QAction(QIcon(":/Icons/bike_new.png"), tr("&New .kmc"), menu);
  newAct->setShortcut(tr("Shift+N"));
  newAct->setStatusTip(tr("Create a new Bike motion file"));

  saveAct = new QAction(QIcon(":/Icons/bike_save.png"), tr("&Save .kmc"), menu);
  saveAct->setShortcut(tr("Shift+S"));
  saveAct->setEnabled(!undo.empty());
  saveAct->setStatusTip(tr("Save Bike motion file under its current name"));
  connect(this, SIGNAL(saveAvailable(bool)), saveAct, SLOT(setEnabled(bool)));

  saveAsAct = new QAction(QIcon(":/Icons/bike_save_as.png"), tr("Save .kmc &As"), menu);
  saveAsAct->setShortcut(tr("Shift+Alt+S"));
  saveAsAct->setStatusTip(tr("Save Bike motion file using a new name"));

  loadAct = new QAction(QIcon(":/Icons/bike_open.png"), tr("&Open .kmc"), menu);
  loadAct->setShortcut(tr("Shift+O"));
  loadAct->setStatusTip(tr("Open a Bike motion file"));

  connect(newAct, SIGNAL(triggered()), this, SLOT(newButtonClicked()));
  connect(saveAct, SIGNAL(triggered()), this, SLOT(saveButtonClicked()));
  connect(saveAsAct, SIGNAL(triggered()), this, SLOT(saveAsButtonClicked()));
  connect(loadAct, SIGNAL(triggered()), this, SLOT(loadButtonClicked()));

  menu->addAction(newAct);
  menu->addAction(loadAct);
  menu->addAction(saveAct);
  menu->addAction(saveAsAct);

  return menu;
}

QMenu* ViewBikeHeaderedWidget::createEditMenu() const {
  QMenu* menu = new QMenu(tr("&BikeEdit"));

  QAction *undoAct, *redoAct, *singleDraw, *reachedDraw, *show3D, *showEditor, *show1D, *show2D, *showVelo, *showAccel,
    *noExtraView, *followMode;

  undoAct = new QAction(QIcon(":/Icons/arrow_undo.png"), tr("Undo"), menu);
  undoAct->setShortcut(QKeySequence::Undo);
  undoAct->setStatusTip(tr("Undo last change"));
  undoAct->setEnabled(!undo.empty());
  connect(this, SIGNAL(undoAvailable(bool)), undoAct, SLOT(setEnabled(bool)));

  redoAct = new QAction(QIcon(":/Icons/arrow_redo.png"), tr("Redo"), menu);
  redoAct->setShortcut(QKeySequence::Redo);
  redoAct->setStatusTip(tr("Redo last undone change"));
  redoAct->setEnabled(!redo.empty());
  connect(this, SIGNAL(redoAvailable(bool)), redoAct, SLOT(setEnabled(bool)));

  show3D = new QAction(tr("Display Phase Drawings"), menu);
  show3D->setStatusTip(tr("Draws curves for every limb either for the current phase or all phases"));
  show3D->setCheckable(true);

  singleDraw = new QAction(tr("Display Only Current Phase"), menu);
  singleDraw->setStatusTip(tr("Draws only curves for the current Phase"));
  singleDraw->setCheckable(true);

  reachedDraw = new QAction(tr("Display Reached Positions"), menu);
  reachedDraw->setStatusTip(tr("Draws the reached positions into the 3D view"));
  reachedDraw->setCheckable(true);

  showEditor = new QAction(tr("Display Editor View"), menu);
  showEditor->setStatusTip(tr("Shows the editor view"));
  showEditor->setCheckable(true);

  show1D = new QAction(tr("Display 1D Views"), menu);
  show1D->setStatusTip(tr("Shows 1D views of one curve for each axis"));
  show1D->setCheckable(true);

  show2D = new QAction(tr("Display 2D Views"), menu);
  show2D->setStatusTip(tr("Shows 2D views of one curve for plane"));
  show2D->setCheckable(true);

  showVelo = new QAction(tr("Display Velocity Views"), menu);
  showVelo->setStatusTip(tr("Shows the velocity of one curve"));
  showVelo->setCheckable(true);

  showAccel = new QAction(tr("Display Acceleration Views"), menu);
  showAccel->setStatusTip(tr("Shows the acceleration of one curve"));
  showAccel->setCheckable(true);

  noExtraView = new QAction(tr("Display No Extra View"), menu);
  noExtraView->setStatusTip(tr("Hides all extra views"));
  noExtraView->setCheckable(true);

  followMode = new QAction(tr("Enable Follow Mode"), menu);
  followMode->setStatusTip(tr("The robot will react to changes directly"));
  followMode->setCheckable(true);

  connect(undoAct, SIGNAL(triggered()), this, SLOT(undoChanges()));
  connect(redoAct, SIGNAL(triggered()), this, SLOT(redoChanges()));

  connect(singleDraw, SIGNAL(toggled(bool)), viewBikeWidget, SLOT(setSingleDrawing(bool)));
  connect(reachedDraw, SIGNAL(toggled(bool)), viewBikeWidget, SLOT(setReachedDrawing(bool)));
  connect(show3D, SIGNAL(toggled(bool)), viewBikeWidget, SLOT(setDrawings(bool)));
  connect(showEditor, SIGNAL(toggled(bool)), viewBikeWidget, SLOT(setEditor(bool)));
  connect(show1D, SIGNAL(toggled(bool)), viewBikeWidget, SLOT(setTra1d(bool)));
  connect(show2D, SIGNAL(toggled(bool)), viewBikeWidget, SLOT(setTra2d(bool)));
  connect(showVelo, SIGNAL(toggled(bool)), viewBikeWidget, SLOT(setVelocity(bool)));
  connect(showAccel, SIGNAL(toggled(bool)), viewBikeWidget, SLOT(setAccel(bool)));
  connect(followMode, SIGNAL(toggled(bool)), viewBikeWidget, SLOT(setFollowMode(bool)));

  QActionGroup* showing = new QActionGroup(menu);
  showing->addAction(show1D);
  showing->addAction(show2D);
  showing->addAction(showVelo);
  showing->addAction(showAccel);
  showing->addAction(noExtraView);

  menu->addAction(undoAct);
  menu->addAction(redoAct);
  menu->addSeparator();
  menu->addAction(show3D);
  menu->addAction(singleDraw);
  menu->addAction(reachedDraw);
  menu->addAction(showEditor);
  menu->addSeparator();
  menu->addAction(noExtraView);
  menu->addAction(show1D);
  menu->addAction(show2D);
  menu->addAction(showVelo);
  menu->addAction(showAccel);
  menu->addSeparator();
  menu->addAction(followMode);

  showEditor->setChecked(true);
  show3D->setChecked(true);
  noExtraView->setChecked(true);

  return menu;
}

bool ViewBikeHeaderedWidget::canClose() {
  if (undo.empty()) {
    return true;
  }
  switch (QMessageBox::warning(this,
                               tr("BikeView"),
                               tr("Do you want to save changes to %1?").arg(fileName),
                               QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel)) {
  case QMessageBox::Save:
    saveButtonClicked();
    break;
  case QMessageBox::Discard:
    break;
  default:
    return false;
  }
  return true;
}

void ViewBikeHeaderedWidget::newButtonClicked() {
  parameters.phaseParameters.clear();
  strcpy(parameters.name, "newKick");
  fileName = QString("newKick.kmc");
  parameters.numberOfPhases = 0;
  parameters.footOrigin = Vector3<>(0.f, 60.f, -210.f);
  parameters.armOrigin = Vector3<>(0.f, 100.f, 30.f);
  parameters.kdx = 0;
  parameters.kix = 0;
  parameters.kpx = 0;
  parameters.kdy = 0;
  parameters.kiy = 0;
  parameters.kpy = 0;

  parameters.loop = false;
  parameters.standLeft = true;
  parameters.adjustKickFootPosition = -1;
  parameters.comOrigin = Vector2<>(10.f, 0.f);
  parameters.ignoreHead = false;

  undo.clear();
  redo.clear();
  emit undoAvailable(false);
  emit redoAvailable(false);
  viewBikeWidget->updateEditorView();
}

void ViewBikeHeaderedWidget::loadButtonClicked() {
  char dirname[260];
  sprintf(dirname, "%s/Config/Kicks/", File::getBHDir());
  fileName = QFileDialog::getOpenFileName(this, tr("Open Bike Motion"), dirname, tr("Bike Motion Config Files (*.kmc)"));
  QString name;
  name = fileName.remove(0, fileName.lastIndexOf("/", fileName.lastIndexOf("/") - 1) + 1);

  InMapFile stream(name.toUtf8().constData());
  if (stream.exists()) {
    stream >> parameters;
    name = name.remove(0, name.lastIndexOf("/") + 1);
    strcpy(parameters.name, name.remove(name.lastIndexOf("."), name.length()).toUtf8().constData());

    parameters.initFirstPhase();
    viewBikeWidget->updateEditorView();
    undo.clear();
    redo.clear();
    emit undoAvailable(false);
    emit redoAvailable(false);
    emit saveAvailable(false);
  }
}

void ViewBikeHeaderedWidget::saveAsButtonClicked() {
  char dirname[260];
  sprintf(dirname, "%s/Config/Kicks/", File::getBHDir());
  fileName =
    QFileDialog::getSaveFileName(this, tr("Save Bike Motion as..."), dirname, tr("Bike Motion Config Files (*.kmc)"));

  if (fileName.begin() != fileName.end()) {
    QString temp = fileName.remove(0, fileName.lastIndexOf("/", fileName.lastIndexOf("/") - 1) + 1);
    temp = temp.remove(0, temp.lastIndexOf("/") + 1);
    strcpy(parameters.name, temp.remove(temp.lastIndexOf("."), temp.length()).toUtf8().constData());
    writeParametersToFile(fileName.toUtf8().constData());
    undo.clear();
    redo.clear();
    emit undoAvailable(false);
    emit redoAvailable(false);
  }
}
void ViewBikeHeaderedWidget::saveButtonClicked() {
  if (fileName.begin() != fileName.end() && fileName != QString("newKick.kmc")) {
    QString name;
    name = fileName.remove(0, fileName.lastIndexOf("/", fileName.lastIndexOf("/") - 1) + 1);
    writeParametersToFile(name.toUtf8().constData());
    undo.clear();
    redo.clear();
    emit undoAvailable(false);
    emit redoAvailable(false);
    emit saveAvailable(false);
  } else {
    saveAsButtonClicked();
  }
}

void ViewBikeHeaderedWidget::writeParametersToFile(const std::string& name) {
  OutTextFile file(name, false);
  file << "footOrigin"
       << "="
       << "{x"
       << "=" << parameters.footOrigin.x << ";"
       << "y"
       << "=" << parameters.footOrigin.y << ";"
       << "z"
       << "=" << parameters.footOrigin.z << ";};" << endl;
  file << "footRotOrigin"
       << "="
       << "{x"
       << "=" << parameters.footRotOrigin.x << ";"
       << "y"
       << "=" << parameters.footRotOrigin.y << ";"
       << "z"
       << "=" << parameters.footRotOrigin.z << ";};" << endl;
  file << "armOrigin"
       << "="
       << "{x"
       << "=" << parameters.armOrigin.x << ";"
       << "y"
       << "=" << parameters.armOrigin.y << ";"
       << "z"
       << "=" << parameters.armOrigin.z << ";};" << endl;
  file << "handRotOrigin"
       << "="
       << "{x"
       << "=" << parameters.handRotOrigin.x << ";"
       << "y"
       << "=" << parameters.handRotOrigin.y << ";"
       << "z"
       << "=" << parameters.handRotOrigin.z << ";};" << endl;
  file << "comOrigin"
       << "="
       << "{x"
       << "=" << parameters.comOrigin.x << ";"
       << "y"
       << "=" << parameters.comOrigin.y << ";};" << endl;
  file << "headOrigin"
       << "="
       << "{x"
       << "=" << parameters.headOrigin.x << ";"
       << "y"
       << "=" << parameters.comOrigin.y << ";};" << endl;
  file << "kpx"
       << "=" << parameters.kpx << ";" << endl;
  file << "kix"
       << "=" << parameters.kix << ";" << endl;
  file << "kdx"
       << "=" << parameters.kdx << ";" << endl;
  file << "kpy"
       << "=" << parameters.kpy << ";" << endl;
  file << "kiy"
       << "=" << parameters.kiy << ";" << endl;
  file << "kdy"
       << "=" << parameters.kdy << ";" << endl;
  if (parameters.loop) {
    file << "loop"
         << "="
         << "true"
         << ";" << endl;
  } else {
    file << "loop"
         << "="
         << "false"
         << ";" << endl;
  }
  if (parameters.standLeft) {
    file << "standLeft"
         << "="
         << "true"
         << ";" << endl;
  } else {
    file << "standLeft"
         << "="
         << "false"
         << ";" << endl;
  }

  file << "adjustKickFootPosition"
       << "=" << parameters.adjustKickFootPosition << ";" << endl;

  if (parameters.ignoreHead) {
    file << "ignoreHead"
         << "="
         << "true"
         << ";" << endl;
  } else {
    file << "ignoreHead"
         << "="
         << "false"
         << ";" << endl;
  }

  file << "numberOfPhases"
       << "=" << parameters.numberOfPhases << ";" << endl;
  file << "phaseParameters"
       << "="
       << "[" << endl;

  for (int i = 0; i < parameters.numberOfPhases; i++) {
    file << "{" << endl;
    file << "duration"
         << "=" << parameters.phaseParameters[i].duration << ";" << endl;
    file << "leftFootTra1"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][1].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][1].y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][1].z << ";};" << endl;
    file << "leftFootTra2"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][2].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][2].y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][2].z << ";};" << endl;
    file << "leftFootRot1"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][1].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][1].y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][1].z << ";};" << endl;
    file << "leftFootRot2"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][2].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][2].y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][2].z << ";};" << endl;
    file << "rightFootTra1"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][1].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][1].y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][1].z << ";};" << endl;
    file << "rightFootTra2"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][2].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][2].y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][2].z << ";};" << endl;
    file << "rightFootRot1"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][1].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][1].y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][1].z << ";};" << endl;
    file << "rightFootRot2"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][2].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][2].y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][2].z << ";};" << endl;

    file << "leftArmTra1"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][1].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][1].y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][1].z << ";};" << endl;
    file << "leftArmTra2"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][2].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][2].y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][2].z << ";};" << endl;
    file << "leftHandRot1"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][1].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][1].y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][1].z << ";};" << endl;
    file << "leftHandRot2"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][2].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][2].y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][2].z << ";};" << endl;

    file << "rightArmTra1"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][1].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][1].y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][1].z << ";};" << endl;
    file << "rightArmTra2"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][2].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][2].y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][2].z << ";};" << endl;
    file << "rightHandRot1"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][1].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][1].y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][1].z << ";};" << endl;
    file << "rightHandRot2"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][2].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][2].y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][2].z << ";};" << endl;

    file << "comTra1"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].comTra[1].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].comTra[1].y << ";};" << endl;
    file << "comTra2"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].comTra[2].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].comTra[2].y << ";};" << endl;

    file << "headTra1"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].headTra[1].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].headTra[1].y << ";};" << endl;
    file << "headTra2"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].headTra[2].x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].headTra[2].y << ";};" << endl;

    file << "odometryOffset"
         << "="
         << "{x"
         << "=" << parameters.phaseParameters[i].odometryOffset.x << ";"
         << "y"
         << "=" << parameters.phaseParameters[i].odometryOffset.y << ";"
         << "z"
         << "=" << parameters.phaseParameters[i].odometryOffset.z << ";};" << endl;

    if (i < parameters.numberOfPhases - 1) {
      file << "}," << endl;
    } else {
      file << "}" << endl;
    }
  }

  file << "];" << endl;
}

void ViewBikeHeaderedWidget::addStateToUndoList() {

  undo.push_back(parameters);

  emit undoAvailable(true);
  if (fileName.size() > 0 && fileName != QString("newKick.kmc")) {
    emit saveAvailable(true);
  }

  if (undo.size() >= 20) {
    undo.erase(undo.begin());
  }
}

void ViewBikeHeaderedWidget::undoChanges() {

  if (!undo.empty()) {
    KickEngineParameters last;
    last = undo.back();
    undo.pop_back();
    redo.push_back(parameters);
    emit redoAvailable(true);

    if (redo.size() >= 20) {
      redo.erase(redo.begin());
    }
    if (parameters.numberOfPhases == last.numberOfPhases) {
      parameters = last;
      viewBikeWidget->updateCommon();
      for (int i = 0; i < parameters.numberOfPhases; i++) {
        viewBikeWidget->fillModelWithPhaseData(i);
      }
    } else {
      parameters = last;
      viewBikeWidget->updateEditorView();
    }
  }

  if (undo.empty()) {
    emit undoAvailable(false);
    emit saveAvailable(false);
  }
}

void ViewBikeHeaderedWidget::redoChanges() {

  if (!redo.empty()) {
    KickEngineParameters last;
    last = redo.back();
    redo.pop_back();

    undo.push_back(parameters);
    emit undoAvailable(true);
    if (fileName.begin() != fileName.end() && fileName != QString("newKick.kmc")) {
      emit saveAvailable(true);
    }

    if (undo.size() >= 20) {
      undo.erase(undo.begin());
    }

    if (parameters.numberOfPhases == last.numberOfPhases) {
      parameters = last;
      viewBikeWidget->updateCommon();
      for (int i = 0; i < parameters.numberOfPhases; i++) {
        viewBikeWidget->fillModelWithPhaseData(i);
      }
    } else {
      parameters = last;
      viewBikeWidget->updateEditorView();
    }
    viewBikeWidget->updateGL();
  }

  if (redo.empty()) {
    emit redoAvailable(false);
  }
}

ViewBike::ViewBike(const QString& fullName,
                   RobotConsole& console,
                   const MotionRequest& motionRequest,
                   const JointData& jointData,
                   const JointCalibration& jc,
                   const SensorData& sensorData,
                   const RobotDimensions& rd,
                   const std::string& mr,
                   SimRobotCore2::Body* robot)
    : fullName(fullName), console(console), motionRequest(motionRequest), jointData(jointData), jointCalibration(jc),
      sensorData(sensorData), robotDimensions(rd), motionRequestCommand(mr), robot(robot) {}

SimRobot::Widget* ViewBike::createWidget() {
  return new ViewBikeHeaderedWidget(*this);
}
