/**
 * @file Global.cpp
 *
 * Implementation of a class that contains pointers to global data.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "Core/Global.h"

PROCESS_WIDE_STORAGE(OutMessage) Global::theDebugOut = 0;
PROCESS_WIDE_STORAGE(OutMessage) Global::theTeamOut = 0;
PROCESS_WIDE_STORAGE(Settings) Global::theSettings = 0;
PROCESS_WIDE_STORAGE(DebugRequestTable) Global::theDebugRequestTable = 0;
PROCESS_WIDE_STORAGE(DebugDataTable) Global::theDebugDataTable = 0;
PROCESS_WIDE_STORAGE(StreamHandler) Global::theStreamHandler = 0;
PROCESS_WIDE_STORAGE(DrawingManager) Global::theDrawingManager = 0;
PROCESS_WIDE_STORAGE(DrawingManager3D) Global::theDrawingManager3D = 0;
PROCESS_WIDE_STORAGE(TimingManager) Global::theTimingManager = 0;
