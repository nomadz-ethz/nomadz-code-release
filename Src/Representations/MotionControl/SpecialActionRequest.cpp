/**
 * @file SpecialActionRequest.cpp
 *
 * This file implements a class to represent special action requests.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#include <cstring>

#include "SpecialActionRequest.h"

SpecialActionRequest::SpecialActionID SpecialActionRequest::getSpecialActionFromName(const char* name) {
  for (int i = 0; i < numOfSpecialActionIDs; ++i) {
    if (!strcmp(name, getName(SpecialActionID(i)))) {
      return SpecialActionID(i);
    }
  }
  return numOfSpecialActionIDs;
}