/**
 * @file Libraries.h
 *
 * The file declares a class that instantiates all libraries.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "LibraryBase.h"

namespace behavior {
#include "Libraries/LibCodeRelease.h"
#include "Libraries/LibLost.h"
#include "Libraries/LibWorldModel.h"

  class Libraries : public BehaviorBase {
  private:
    static PROCESS_WIDE_STORAGE(Libraries) theInstance;
    std::vector<LibraryBase*> libraries; /**< All the member libraries of this class. */

  public:
    LibCodeRelease libCodeRelease;
    LibLost libLost;
    LibWorldModel libWorldModel;

    Libraries(internal::ref_init_tag t) : BehaviorBase(t), libCodeRelease(t), libLost(t), libWorldModel(t) {}

    Libraries(const BehaviorControlBase& base, BehaviorControlOutput& behaviorControlOutput);

    /**
     * Assignment operator, because the standard operator is not accepted by the compiler.
     * @param other The instance that is cloned.
     */
    void operator=(const Libraries& other);

    /** Calls the preProcess() method of each member library */
    void preProcessLibraries();

    /** Calls the postProcess() method of each member library */
    void postProcessLibraries();

    friend class LibraryBase;
  };
} // namespace behavior
