/**
 * @file Stopwatch.h
 *
 * The file declares the stopwatch macros.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</a>,
 * <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>,
 * and <a href="mailto:arneboe@tzi.de">Arne Böckmann</a>
 */

#pragma once
#include "TimingManager.h"
#include "DebugDrawings.h"
#include "Debugging.h"

/*
 * Allows for the measurement of time
 * @param eventID The id of the stop watch
 * @param expression The expression of which the execution time is measured
 */
#define STOP_TIME_ON_REQUEST(eventID, expression)                                                                           \
  do {                                                                                                                      \
    TimingManager& tm = Global::getTimingManager();                                                                         \
    tm.startTiming(eventID);                                                                                                \
    {expression} tm.stopTiming(eventID);                                                                                    \
  } while (false)

#define STOP_TIME_ON_REQUEST_WITH_PLOT(eventID, expression)                                                                 \
  do {                                                                                                                      \
    DECLARE_PLOT("stopwatch:" eventID);                                                                                     \
    TimingManager& tm = Global::getTimingManager();                                                                         \
    tm.startTiming(eventID);                                                                                                \
    { expression }                                                                                                          \
    const unsigned time = tm.stopTiming(eventID);                                                                           \
    PLOT("stopwatch:" eventID, time * 0.001f);                                                                              \
  } while (false)
