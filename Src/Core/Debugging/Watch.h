/**
 * @file Watch.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in License.Nao_Devils.txt.
 * (c) 2023 Nao Devils and NomadZ team
 */

#pragma once

#include "Core/System/BHAssert.h"

#define WATCH(x) ASSERT(x == x);
#define WATCH_ARRAY(a)                                                                                                      \
  for (unsigned int i = 0; i < sizeof(a) / sizeof(a[0]); i++)                                                               \
    ASSERT(a[i] == a[i]);

class Watch {
public:
  virtual void watch(){};
};
