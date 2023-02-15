/**
 * @file Cognition.h
 *
 * Declaration of a class that represents a process that receives data from the robot at about 60 Hz.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#pragma once

#include "Core/ProcessFramework/Process.h"
#include "Core/Module/ModulePackage.h"
#include "Core/ProcessFramework/TeamHandler.h"
#include "Tools/Logging/CognitionLogger.h"

/**
 * @class Cognition
 * A class that represents a process that receives data from the robot at about 30 Hz.
 */
class Cognition : public Process {
private:
  DEBUGGING;
  RECEIVER(MotionToCognition);
  SENDER(CognitionToMotion);
  TEAM_COMM;

public:
  /**
   * Default constructor.
   */
  Cognition();

  /**
   * Default destructor.
   * Sets the global pointers again (only for Simulator).
   */
  ~Cognition() { setGlobals(); }

  /**
   * The method is called from the framework once in every frame.
   */
  virtual bool main();

  /**
   * The method is called directly before the first call of main().
   */
  virtual void init();

  /**
   * The method is called when the process is terminated.
   */
  virtual void terminate();

  /**
   * The function handles incoming debug messages.
   * @param message the message to handle.
   * @return Has the message been handled?
   */
  virtual bool handleMessage(InMessage& message);

  ModuleManager moduleManager; /**< The solution manager handles the execution of modules. */
  CognitionLogger logger;
};
