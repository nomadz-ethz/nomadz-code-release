/**
 * @file LibraryBase.h
 *
 * The file declares the base class for all behavior libraries.
 * If a library is used by another library, it must be added here.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Thomas Röfer
 */

#pragma once

#include "BehaviorControl.h"

namespace behavior {
  class LibCodeRelease;

  class LibraryBase : public BehaviorBase {
  public:
    LibCodeRelease& libCodeRelease;
    /**
     * The default constructor initializes all references with the actual libraries.
     * Note that these libraries may not be initialzed yet, so do not call them
     * during construction.
     */
    LibraryBase();

    LibraryBase(internal::ref_init_tag t) : BehaviorBase(t), libCodeRelease(libCodeRelease) {}

    /**
     * The method is called each time before a behavior cycle is executed.
     * It is supposed to be overridden.
     */
    virtual void preProcess() {}

    /**
     * The method is called each time after a behavior cycle was executed.
     * It is supposed to be overridden.
     */
    virtual void postProcess() {}
  };
} // namespace behavior
