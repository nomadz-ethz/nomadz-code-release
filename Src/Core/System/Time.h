/**
 * @file Time.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#pragma once
#include <climits>

class Time {
private:
  static unsigned long long base; /**< An offset used to convert the time to the time provided by this class. */

#ifndef TARGET_ROBOT
  static bool isInitialized;   /**< Whether the system time is initialized. */
  static bool isTimeSimulated; /**< Whether only simulated time is used. */
  static int simulatedTime;    /**< The offset of the simulated time. */
#endif

public:
  /** returns the current system time in milliseconds*/
  static unsigned getCurrentSystemTime();

  /** returns the real system time in milliseconds (never the simulated one)*/
  static unsigned getRealSystemTime();

  /** returns an offset used to convert the time to the time provided by this class. */
  static unsigned getSystemTimeBase();

  /**
   * The function returns the thread cpu time of the calling thread in microseconds.
   * return thread cpu time of the calling thread
   */
  static unsigned long long getCurrentThreadTime();

  /** returns the time since aTime*/
  static int getTimeSince(unsigned aTime);

  /** returns the real time since aTime*/
  static int getRealTimeSince(unsigned aTime);

#ifndef TARGET_ROBOT
  /** Initializes the system time. */
  static void initialize();

  /** Deinitializes the system time. */
  static void deinitialize();

  /** Switches simulated time on/off. */
  static void setSimulatedTime(bool on);

  /** Adds dt to the simulated time. */
  static void addSimulatedTime(int dt);
#endif
};

inline unsigned Time::getSystemTimeBase() {
  if (!base)
    static_cast<void>(getRealSystemTime());
  return base;
}

inline int Time::getTimeSince(unsigned aTime) {
  if (aTime != INT_MAX) {
    return static_cast<int>(getCurrentSystemTime() - aTime);
  } else {
    return INT_MAX;
  }
}

inline int Time::getRealTimeSince(unsigned aTime) {
  if (aTime != INT_MAX) {
    return static_cast<int>(getRealSystemTime() - aTime);
  } else {
    return INT_MAX;
  }
}
