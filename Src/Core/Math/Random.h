/**
 * @file Random.h
 *
 * This contains some functions for creating random numbers.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * and Max Risler
 */

#pragma once

#include <cstdlib>

/**
 * The function returns a random number in the range of [0..1].
 * @return The random number.
 */
static inline float randomFloat() {
  return float(rand()) * (1.f / static_cast<float>(RAND_MAX));
}

/**
 * The function returns a random integer number in the range of [0..n-1].
 * @param n the number of possible return values (0 ... n-1)
 * @return The random number.
 */
static inline int random(int n) {
  return int((long long)rand() * n / ((unsigned int)RAND_MAX + 1));
}

/**
 * The function returns a random integer number in the range of [0..n-1].
 * @param n the number of possible return values (0 ... n-1)
 * @return The random number.
 */
static inline int random(short n) {
#if RAND_MAX < 0xffff
  return rand() * n / (RAND_MAX + 1);
#else
  return int((long long)rand() * n / ((unsigned int)RAND_MAX + 1));
#endif
}
