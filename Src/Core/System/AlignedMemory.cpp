/**
 * @file AlignedMemory.cpp
 *
 * Base class for all types that need aligned allocations. This is required
 * whenever they contain Eigen attributes.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "AlignedMemory.h"
#include "SystemCall.h"

void* AlignedMemory::operator new(std::size_t size) {
  return SystemCall::alignedMalloc(size);
}

void* AlignedMemory::operator new[](std::size_t size) {
  return SystemCall::alignedMalloc(size);
}

void AlignedMemory::operator delete(void* ptr) throw() {
  SystemCall::alignedFree(ptr);
}

void AlignedMemory::operator delete[](void* ptr) throw() {
  SystemCall::alignedFree(ptr);
}

void* AlignedMemory::operator new(std::size_t size, const std::nothrow_t&) throw() {
  return SystemCall::alignedMalloc(size);
}

void AlignedMemory::operator delete(void* ptr, const std::nothrow_t&) throw() {
  SystemCall::alignedFree(ptr);
}
