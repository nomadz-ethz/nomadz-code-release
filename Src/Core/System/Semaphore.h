/**
 * @file Semaphore.h
 *
 * Declaration of class Semaphore for thread synchronization.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#pragma once

#define SEMAPHORE_INCLUDED

/**
 * Encapsulates an semaphore object.
 */
class Semaphore {
public:
  /**
   * Constructs an new semaphore object.
   * @param value The initial value for the semaphore.
   */
  Semaphore(unsigned int value = 0);

  /** Destructor. */
  ~Semaphore();

  /**
   * Increments the semaphore counter.
   */
  void post();

  /**
   * Decrements the semaphore counter. This function returns immediatly if the
   * counter is greater than zero. Otherwise the wait call blocks until the semaphore
   * counter can be decremented.
   * @return Whether the decrementation was successful.
   */
  bool wait();

  /**
   * Decrements the semaphore counter. This function returns immediatly if the
   * counter is greater than zero. Otherwise the wait call blocks until the semaphore
   * counter can be decremented.
   * @param timeout A timeout for the blocking call. (in ms)
   * @return Whether the decrementation was successful.
   */
  bool wait(unsigned int timeout);

  /**
   * Tries to decrement the semaphore counter. This function returns immediatly.
   * @return Whether the decrementation was successful.
   */
  bool tryWait();

private:
  void* handle; /**< The sem_t handle of the semaphore. */
  char handle2[sizeof(void*) * 4];
};
