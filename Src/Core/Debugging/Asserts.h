/**
 * @file Asserts.h
 *
 * Some more complex assertions based on the ASSERT macro.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */

#pragma once

#include "Core/System/BHAssert.h"
#include <cmath>

#define ASSERT_EQUALS_DELTA(actual, expected, delta) ASSERT(fabs((actual) - (expected)) <= (delta));

#define ASSERT_WITHIN(n, begin, end)                                                                                        \
  ASSERT((n) >= begin);                                                                                                     \
  ASSERT((n) <= end);

#define ASSERT_INDEX_WITHIN(i, begin, end)                                                                                  \
  ASSERT((i) >= begin);                                                                                                     \
  ASSERT((i) < end);

#define ASSERT_COORDINATES_WITHIN_IMAGE(x, y, image)                                                                        \
  ASSERT_INDEX_WITHIN(x, 0, (image).width);                                                                                 \
  ASSERT_INDEX_WITHIN(y, 0, (image).height);

#define ASSERT_POINT_WITHIN_IMAGE(p, image) ASSERT_COORDINATES_WITHIN_IMAGE((p).x, (p).y, image)
