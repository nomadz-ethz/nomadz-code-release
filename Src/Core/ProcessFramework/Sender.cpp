/**
 * @file Sender.cpp
 *
 * This file implements classes related to senders.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a> */
#include "ProcessFramework.h"

SenderList::SenderList(PlatformProcess* p, const std::string& senderName)
    : next(0), name(senderName), // copy the sender's name. The name of the process is still missing.
      process(p) {
  if (getFirst()) {
    SenderList* p = getFirst();
    while (p->next) {
      p = p->next;
    }
    p->next = this;
  } else {
    getFirst() = this;
  }
}

SenderList*& SenderList::getFirst() {
  return process->getFirstSender();
}

void SenderList::sendAllUnsentPackages() {
  for (SenderList* p = getFirst(); p; p = p->getNext()) {
    p->sendPackage();
  }
}

SenderList* SenderList::lookup(const std::string& processName, const std::string& senderName) {
  for (SenderList* p = getFirst(); p; p = p->getNext()) {
    if (processName + "." + p->name == senderName) {
      return p;
    }
  }
  return 0;
}
