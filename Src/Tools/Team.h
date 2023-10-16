/**
 * @file Team.h
 *
 * Declaration of a class that contains pointers to global data.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author of this file is Thomas Röfer.
 */

#pragma once

#include "Core/MessageQueue/OutMessage.h"
#include "Core/Global.h"

/**
 * A macro for broadcasting team messages.
 * @param type The type of the message from the MessageID enum in MessageIDs.h
 * @param format The message format of the message (bin or text).
 * @param expression A streamable expression.
 */
#define TEAM_OUTPUT(type, format, expression)                                                                               \
  {                                                                                                                         \
    Global::getTeamOut().format << expression;                                                                              \
    Global::getTeamOut().finishMessage(type);                                                                               \
  }

/**
 * A macro for broadcasting team messages. This macro will only write data if it
 * will be send in the current frame. This requires the representation
 * TeamMateData.
 * @param type The type of the message from the MessageID enum in MessageIDs.h
 * @param format The message format of the message (bin or text).
 * @param expression A streamable expression.
 */
#define TEAM_OUTPUT_FAST(type, format, expression)                                                                          \
  if (theTeamMateData.sendThisFrame) {                                                                                      \
    Global::getTeamOut().format << expression;                                                                              \
    Global::getTeamOut().finishMessage(type);                                                                               \
  }
