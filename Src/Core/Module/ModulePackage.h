/**
 * @file ModulePackage.h
 *
 * Declaration of a class representing a package transmitted between different processes.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "ModuleManager.h"

/**
 * @class ModulePackage
 * A class representing a package transmitted between different processes.
 */
class ModulePackage {
public:
  ModuleManager* moduleManager; /**< A pointer to the module manager. It knows the actual data to be streamed. */
  unsigned timeStamp;           /**< The time stamp of the package. */

  /**
   * Default constructor.
   */
  ModulePackage() : moduleManager(0), timeStamp(0) {}
};

/**
 * The operator will use the module manager to write the required blackboard entries
 * to the stream.
 * @param stream The stream that is written to.
 * @param modulePackage The package associated to the module manager.
 * @return The stream.
 */
inline Out& operator<<(Out& stream, const ModulePackage& modulePackage) {
  stream << modulePackage.timeStamp;
  modulePackage.moduleManager->writePackage(stream);
  return stream;
}

/**
 * The operator will use the module manager to read the required blackboard entries
 * from the stream.
 * @param stream The stream that is read from.
 * @param modulePackage The package associated to the module manager.
 * @return The stream.
 */
inline In& operator>>(In& stream, ModulePackage& modulePackage) {
  stream >> modulePackage.timeStamp;
  modulePackage.moduleManager->readPackage(stream);
  return stream;
}

class CognitionToMotion : public ModulePackage {};
class MotionToCognition : public ModulePackage {};
