/**
 * @file Annotator.h
 *
 * Declaration of Annotator classes
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <memory>
#include <vector>
#include <QObject>
#include "Controller/ConsoleRoboCupCtrl.h"
#include "Controller/Representations/Lesson.h"
#include "Controller/ControllerRobot.h"

class Annotator : public QObject {
public:
  virtual ~Annotator(){};
  virtual std::vector<std::unique_ptr<Annotation>> annotate(const std::vector<DataFrame*>& frames) = 0;

  virtual bool start(const std::vector<DataFrame*>& frames) = 0;
  virtual std::vector<std::unique_ptr<Annotation>> giveResults() = 0;

signals:
  // Never use a direct connection to progressed or finished; always use a queued connection.
  // (Direct connections can cause deadlocks if the slot is not careful.)
  void progressed(int, int);
  void finished();

private:
  Q_OBJECT
};

class RoboticAnnotator : public Annotator {
public:
  RoboticAnnotator(ConsoleRoboCupCtrl& console) : console(console) {}

  void spawnRobot(std::string name = "Annotator");

  std::vector<std::unique_ptr<Annotation>> annotate(const std::vector<DataFrame*>& frames) override;

  bool start(const std::vector<DataFrame*>& frames) override;
  std::vector<std::unique_ptr<Annotation>> giveResults() override;

  void handleMessage(InMessage&);

private:
  ConsoleRoboCupCtrl& console;

  ControllerRobot* robot; // non-owning pointer; owner is (usually) ConsoleRoboCupCtrl

  std::map<int, DataFrame*> toProcess; // remaining dataframes to process
  std::vector<std::pair<DataFrame*, std::unique_ptr<DataFrame>>>
    receivedFrames; // all received frames, matched with their source DataFrame

  bool sendFrame(const DataFrame&, bool lock = false);
  std::unique_ptr<DataFrame> incomingFrame; // buffer for frame currently being received
};
