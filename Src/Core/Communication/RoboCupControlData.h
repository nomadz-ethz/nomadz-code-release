/**
 * @file RoboCupControlData.h
 *
 * The file encapsulates definitions in the file RoboCupGameControlData.h
 * that is provided with the GameController in a namespace.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include <cstdint>
#include <cstring>

namespace RoboCup {
#define fieldPlayerColour teamColor
#include "RoboCupGameControlData.h"
#undef fieldPlayerColour
} // namespace RoboCup
