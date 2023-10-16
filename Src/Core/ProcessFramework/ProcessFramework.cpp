/**
 * @file ProcessFramework.cpp
 *
 * This file implements classes corresponding to the process framework.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "ProcessFramework.h"

ProcessCreatorBase* ProcessCreatorBase::first = 0;

void PlatformProcess::setPriority(int priority) {
  this->priority = priority;
  if (processBase) {
    processBase->setPriority(priority);
  }
}
