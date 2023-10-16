/**
 * @file ControllerRobot.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 */

#include <QString>
#include <QFileInfo>

#include "Controller/LocalRobot.h"
#include "Controller/ControllerRobot.h"
#include "Core/Streams/InStreams.h"
#include "Controller/ConsoleRoboCupCtrl.h"
#include "Controller/TeamComm3DCtrl.h"

extern "C" DLL_EXPORT SimRobot::Module* createModule(SimRobot::Application& simRobot) {
  QFileInfo info(simRobot.getFilePath());
  QString baseName = info.baseName();
  if (baseName.startsWith("TeamComm3D")) {
    return new TeamComm3DCtrl(simRobot);
  } else {
    return new ConsoleRoboCupCtrl(simRobot);
  }
}

MAKE_PROCESS(LocalRobot);

ControllerRobot::ControllerRobot(const char* name) : name(name) {
#ifdef ENABLE_ROS
  InMapFile cm("Processes/connectROS.cfg");
#else
  InMapFile cm("Processes/connect.cfg");
#endif
  ASSERT(cm.exists());

  // attach receivers to senders
  ConnectionParameter cp;
  cm >> cp;
  for (std::vector<ConnectionParameter::ProcessConnection>::const_iterator it = cp.processConnections.begin();
       it != cp.processConnections.end();
       ++it) {
    connect(getSender(it->sender.c_str()), getReceiver(it->receiver.c_str()));
  }

  // connect additional senders and receivers
  connect(getSender("Debug.Sender.MessageQueue.S"), getReceiver("LocalRobot.Receiver.MessageQueue.O"));
  connect(getSender("LocalRobot.Sender.MessageQueue.S"), getReceiver("Debug.Receiver.MessageQueue.O"));

  robotProcess = 0;
  for (ProcessList::const_iterator i = begin(); i != end() && !robotProcess; ++i) {
    robotProcess = (LocalRobot*)(*i)->getProcess("LocalRobot");
  }
  ASSERT(robotProcess);
}

void ControllerRobot::update() {
  robotProcess->update();
}

std::string ControllerRobot::getModel() const {
  return "Nao";
}

void ControllerRobot::connect(SenderList* sender, ReceiverList* receiver) {
  if (sender && receiver) {
    sender->add(receiver);
  }
}

SenderList* ControllerRobot::getSender(const std::string& senderName) {
  for (ProcessList::const_iterator i = begin(); i != end(); ++i) {
    std::string name;
    if (senderName[0] == '.') {
      name = (*i)->getName();
    } else {
      name = "";
    }
    name += senderName;
    SenderList* sender = (*i)->lookupSender(name);
    if (sender) {
      return sender;
    }
  }
  TRACE("%s not found", senderName.c_str());
  return 0;
}

ReceiverList* ControllerRobot::getReceiver(const std::string& receiverName) {
  for (ProcessList::const_iterator i = begin(); i != end(); ++i) {
    ReceiverList* receiver = (*i)->lookupReceiver(receiverName);
    if (receiver) {
      return receiver;
    }
  }
  TRACE("%s not found", receiverName.c_str());
  return 0;
}
