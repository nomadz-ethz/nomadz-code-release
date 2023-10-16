/**
 * @file BHToolBar.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 */

#include "BHToolBar.h"
#include "ConsoleRoboCupCtrl.h"
#include "Core/System/SystemCall.h"
#include <QTimer>
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Infrastructure/KeyStates.h"

QMenu* BHToolBar::createUserMenu() const {
  QMenu* menu = new QMenu(tr("NomadZ"));
  QAction* pressChestButtonAct = new QAction(QIcon(":/Icons/chestButton.png"), tr("&Press and Release Chest Button"), menu);
  connect(pressChestButtonAct, SIGNAL(triggered()), this, SLOT(pressChestButton()));
  menu->addAction(pressChestButtonAct);
  return menu;
}

void BHToolBar::stand() {
  setPlayDead();
  QTimer::singleShot(300, this, SLOT(setStand()));
}

void BHToolBar::sitDown() {
  setSitDown();
  QTimer::singleShot(4000, this, SLOT(setPlayDead()));
}

void BHToolBar::setPlayDead() {

  MotionRequest moReq;
  moReq.motion = MotionRequest::Motion::specialAction;
  moReq.specialActionRequest.specialAction = SpecialActionRequest::SpecialActionID::playDead;

  console.setRepresentation("MotionRequest", moReq);
}

void BHToolBar::setSitDown() {
  MotionRequest moReq;
  moReq.motion = MotionRequest::Motion::specialAction;
  moReq.specialActionRequest.specialAction = SpecialActionRequest::SpecialActionID::sitDown;

  console.setRepresentation("MotionRequest", moReq);
}

void BHToolBar::setStand() {
  MotionRequest moReq;
  moReq.motion = MotionRequest::Motion::specialAction;
  moReq.specialActionRequest.specialAction = SpecialActionRequest::SpecialActionID::standHigh;

  console.setRepresentation("MotionRequest", moReq);
}

void BHToolBar::pressChestButton() {
  KeyStates keyState;
  keyState.pressed[KeyStates::Key::chest] = true;

  console.setRepresentation("KeyStates", keyState);

  QTimer::singleShot(80, this, SLOT(releaseChestButton()));
}

void BHToolBar::releaseChestButton() {
  KeyStates keyState;
  keyState.pressed[KeyStates::Key::chest] = false;

  console.setRepresentation("KeyStates", keyState);

  QTimer::singleShot(5, this, SLOT(unchangeButtons()));
}

void BHToolBar::unchangeButtons() {
  console.executeConsoleCommand("set representation:KeyStates unchanged");
}
