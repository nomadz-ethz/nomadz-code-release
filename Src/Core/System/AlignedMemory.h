/**
 * @file AlignedMemory.h
 *
 * Base class for all types that need aligned allocations. This is required
 * whenever they contain Eigen attributes.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include <new>

struct AlignedMemory {
  void* operator new(std::size_t size);
  void* operator new[](std::size_t size);
  void operator delete(void* ptr) throw();
  void operator delete[](void* ptr) throw();
  static void* operator new(std::size_t size, void* ptr) { return ::operator new(size, ptr); }
  static void* operator new[](std::size_t size, void* ptr) { return ::operator new[](size, ptr); }
  void operator delete(void* memory, void* ptr) throw() { return ::operator delete(memory, ptr); }
  void operator delete[](void* memory, void* ptr) throw() { return ::operator delete[](memory, ptr); }
  void* operator new(std::size_t size, const std::nothrow_t&) throw();
  void operator delete(void* ptr, const std::nothrow_t&) throw();
};
