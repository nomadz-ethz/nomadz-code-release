/**
 * @file Libraries.cpp
 *
 * The file implements a class that instantiates all libraries.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include <cstring>
#include "Libraries.h"

namespace behavior {
  PROCESS_WIDE_STORAGE(Libraries) Libraries::theInstance = 0;

  Libraries::Libraries(const BehaviorControlBase& base, BehaviorControlOutput& behaviorControlOutput)
      : BehaviorBase((theInstance = this, base), behaviorControlOutput) {}

  void Libraries::operator=(const Libraries& other) { memcpy((void*)this, &other, sizeof(*this)); }

  void Libraries::preProcessLibraries() {
    for (LibraryBase* library : libraries) {
      library->preProcess();
    }
  }

  void Libraries::postProcessLibraries() {
    for (LibraryBase* library : libraries) {
      library->postProcess();
    }
  }
} // namespace behavior
