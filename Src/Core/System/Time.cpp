/**
 * @file Time.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 */

#include "Core/System/Time.h"
#include "Core/System/BHAssert.h"

#ifndef TARGET_ROBOT
#include <pthread.h>
#endif
#include <time.h>

unsigned long long Time::base = 0;

#ifndef TARGET_ROBOT

bool Time::isInitialized = false;
bool Time::isTimeSimulated = false;
int Time::simulatedTime = 0;

void Time::initialize() {
  simulatedTime = 10000 - getRealSystemTime();
  isTimeSimulated = false;
  isInitialized = true;
}

void Time::deinitialize() {
  isInitialized = false;
}

void Time::setSimulatedTime(bool on) {
  if (isTimeSimulated != on) {
    if (on) {
      simulatedTime += getRealSystemTime();
    } else {
      simulatedTime -= getRealSystemTime();
    }
    isTimeSimulated = on;
  }
}

void Time::addSimulatedTime(int dt) {
  if (isTimeSimulated) {
    simulatedTime += dt;
  }
}

#endif

unsigned Time::getCurrentSystemTime() {
#ifndef TARGET_ROBOT
  if (isInitialized) {
    if (isTimeSimulated) {
      return simulatedTime;
    } else {
      return unsigned(getRealSystemTime() + simulatedTime);
    }
  } else {
    return getRealSystemTime();
  }
#else
  return getRealSystemTime();
#endif
}

unsigned Time::getRealSystemTime() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts); // NTP might change CLOCK_REALTIME on desktop systems
  const unsigned long long time = ts.tv_sec * 1000ll + ts.tv_nsec / 1000000;
  if (!base) {
    base = time - 10000; // avoid time == 0, because it is often used as a marker
  }
  return static_cast<unsigned>(time - base);
}

unsigned long long Time::getCurrentThreadTime() {
  timespec ts;

#ifndef TARGET_ROBOT
  clockid_t cid;
  VERIFY(pthread_getcpuclockid(pthread_self(), &cid) == 0);
  VERIFY(clock_gettime(cid, &ts) == 0);
#else
  VERIFY(clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0);
#endif

  const unsigned long long time = ts.tv_sec * 1000000ll + ts.tv_nsec / 1000;
  if (!base) {
#ifndef TARGET_ROBOT
    base = time - 1000000ll;
#else
    base = time - 100000 * 1000;
#endif
  }
  return time - base;
}
