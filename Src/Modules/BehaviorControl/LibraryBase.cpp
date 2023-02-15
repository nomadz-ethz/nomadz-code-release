/**
 * @file LibraryBase.cpp
 *
 * The file implements the base class for all behavior libraries.
 * If a library is used by another library, it must be added here.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Thomas Röfer
 */

#include "Libraries.h"

namespace behavior {
  LibraryBase::LibraryBase()
      : BehaviorBase(*Libraries::theInstance), libCodeRelease(Libraries::theInstance->libCodeRelease) {
    Libraries::theInstance->libraries.push_back(this);
  }
} // namespace behavior
