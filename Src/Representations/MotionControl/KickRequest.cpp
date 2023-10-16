/**
 * @file KickRequest.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#include <cstring>

#include "KickRequest.h"

KickRequest::KickMotionID KickRequest::getKickMotionFromName(const char* name) {
  for (int i = 0; i < numOfKickMotionIDs; ++i) {
    if (!strcmp(name, getName(KickMotionID(i)))) {
      return KickMotionID(i);
    }
  }
  return numOfKickMotionIDs;
}
