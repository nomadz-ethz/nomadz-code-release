/**
 * @file Global.h
 *
 * Declaration of a class that contains pointers to global data.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Core/System/SystemCall.h"

// Only declare prototypes. Don't include anything here, because this
// file is included in many other files.
class OutMessage;
class Settings;
class DebugRequestTable;
class DebugDataTable;
class StreamHandler;
class DrawingManager;
class DrawingManager3D;
class ReleaseOptions;
class TimingManager;
/**
 * @class Global
 * A class that contains pointers to global data.
 */
class Global {
private:
  static PROCESS_WIDE_STORAGE(OutMessage) theDebugOut;
  static PROCESS_WIDE_STORAGE(OutMessage) theTeamOut;
  static PROCESS_WIDE_STORAGE(Settings) theSettings;
  static PROCESS_WIDE_STORAGE(DebugRequestTable) theDebugRequestTable;
  static PROCESS_WIDE_STORAGE(DebugDataTable) theDebugDataTable;
  static PROCESS_WIDE_STORAGE(StreamHandler) theStreamHandler;
  static PROCESS_WIDE_STORAGE(DrawingManager) theDrawingManager;
  static PROCESS_WIDE_STORAGE(DrawingManager3D) theDrawingManager3D;
  static PROCESS_WIDE_STORAGE(TimingManager) theTimingManager;

public:
  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the outgoing debug message queue in this process.
   */
  static OutMessage& getDebugOut() { return *theDebugOut; }

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the outgoing team message queue in this process.
   */
  static OutMessage& getTeamOut() { return *theTeamOut; }

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the settings in this process.
   */
  static Settings& getSettings() { return *theSettings; }

  /**
   * The method checks for nullptr access, ie. whether the settings exist or not.
   * @return true if settings exist, otherwise false
   */
  static bool settingsExist() { return theSettings != nullptr; }

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the debug request table in this process.
   */
  static DebugRequestTable& getDebugRequestTable() { return *theDebugRequestTable; }

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the debug data table in this process.
   */
  static DebugDataTable& getDebugDataTable() { return *theDebugDataTable; }

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the stream handler in this process.
   */
  static StreamHandler& getStreamHandler() { return *theStreamHandler; }

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the drawing manager in this process.
   */
  static DrawingManager& getDrawingManager() { return *theDrawingManager; }

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the 3-D drawing manager in this process.
   */
  static DrawingManager3D& getDrawingManager3D() { return *theDrawingManager3D; }

  /**
   * The method returns a reference to the process wide instance.
   * @return the instance of the timing manager in this process.
   */
  static TimingManager& getTimingManager() { return *theTimingManager; }

  friend class Process;            // The class Process can set these pointers.
  friend class Cognition;          // The class Cognition can set theTeamOut.
  friend class Settings;           // The class Settings can set a default StreamHandler.
  friend class ConsoleRoboCupCtrl; // The class ConsoleRoboCupCtrl can set theStreamHandler.
  friend class RobotConsole;       // The class RobotConsole can set theDebugOut.
  friend class TeamComm3DCtrl;
  friend class Framework;
  friend class Rf;
};
