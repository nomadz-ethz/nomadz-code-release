/**
 * @file SystemCall.cpp
 *
 * Implementation of system calls and access to thread local storage.
 * Only for use on Linux. The same implementation is also used on the NAO.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are <A href=mailto:brunn@sim.informatik.tu-darmstadt.de>Ronnie Brunn</A>, <A
 * href=mailto:martin@martin-loetzsch.de>Martin Lötzsch</A>, <A href=mailto:risler@sim.informatik.tu-darmstadt.de>Max
 * Risler</A> and <a href=mailto:dueffert@informatik.hu-berlin.de>Uwe Düffert</a>
 */

#include "Core/System/SystemCall.h"
#include "SoundPlayer.h"
#include "BHAssert.h"

#include <sys/sysinfo.h>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <cstring>
#include <pthread.h>
#include <ctime>

#ifdef ENABLE_ROS
using namespace std::literals::chrono_literals;
#include <rclcpp/rclcpp.hpp>
#endif

const char* SystemCall::getHostName() {
  static const char* hostname = 0;
  if (!hostname) {
    static char buf[100] = {0};
    VERIFY(!gethostname(buf, sizeof(buf)));
    hostname = buf;
  }
  return hostname;
}

const char* SystemCall::getHostAddr() {
  static const char* hostaddr = 0;
#ifdef STATIC    // Prevent warnings during static linking
  ASSERT(false); // should not be called
#else
  if (!hostaddr) {
    static char buf[100];
    hostent* hostAddr = (hostent*)gethostbyname(getHostName());
    if (hostAddr && hostAddr->h_addr_list[0])
      strcpy(buf, inet_ntoa(*(in_addr*)hostAddr->h_addr_list[0]));
    else
      strcpy(buf, "127.0.0.1");
    hostaddr = buf;
  }
#endif
  return hostaddr;
}

// When compiling for simulation, the implementation is defined in ConsoleRoboCupCtrl
#ifndef TARGET_SIM
SystemCall::Mode SystemCall::getMode() {
#ifdef TARGET_TOOL
  return teamRobot;
#else
  return physicalRobot;
#endif
}
#endif

void SystemCall::sleep(unsigned ms) {
#ifdef ENABLE_ROS
  rclcpp::sleep_for(ms * 1ms);
#else
  usleep(1000 * ms);
#endif
}

void SystemCall::getLoad(float& mem, float load[3]) {
  struct sysinfo info;
  if (sysinfo(&info) == -1)
    load[0] = load[1] = load[2] = mem = -1.f;
  else {
    load[0] = float(info.loads[0]) / 65536.f;
    load[1] = float(info.loads[1]) / 65536.f;
    load[2] = float(info.loads[2]) / 65536.f;
    mem = float(info.totalram - info.freeram) / float(info.totalram);
  }
}

void* SystemCall::alignedMalloc(size_t size, size_t alignment) {
  void* ptr;
  if (!posix_memalign(&ptr, alignment, size)) {
    return ptr;
  } else {
    return nullptr;
  }
}

void SystemCall::alignedFree(void* ptr) {
  free(ptr);
}

#ifndef TARGET_TOOL
int SystemCall::playSound(const char* name) {
  return SoundPlayer::play(name);
}
#endif
