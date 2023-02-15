/**
 * @file NaoBody.cpp
 *
 * Declaration of a class for accessing the body of the Nao via naobridge.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are Colin Graf
 */

#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>

#include "NaoBody.h"
#include "Core/System/BHAssert.h"
#include "Core/System/SystemCall.h"
#include "Core/System/Time.h"

#include "naobridge/nomadz.h"

class NaoBodyAccess {
public:
  int fd;           /**< The file descriptor for the shared memory block. */
  sem_t* sem;       /**< The semaphore used for synchronizing to the NaoQi DCM. */
  LBHData* lbhData; /**< The pointer to the mapped shared memory block. */

  NaoBodyAccess() : fd(-1), sem(SEM_FAILED), lbhData((LBHData*)MAP_FAILED) {}

  ~NaoBodyAccess() { cleanup(); }

  bool init() {
    if (lbhData != MAP_FAILED)
      return true;

    fd = shm_open(LBH_MEM_NAME, O_RDWR, S_IRUSR | S_IWUSR);
    if (fd == -1)
      return false;

    sem = sem_open(LBH_SEM_NAME, O_RDWR, S_IRUSR | S_IWUSR, 0);
    if (sem == SEM_FAILED) {
      close(fd);
      fd = -1;
      return false;
    }

    VERIFY((lbhData = (LBHData*)mmap(NULL, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)) != MAP_FAILED);
    lbhData->state = okState;
    return true;
  }

  void cleanup() {
    if (lbhData != MAP_FAILED) {
      munmap(lbhData, sizeof(LBHData));
      lbhData = (LBHData*)MAP_FAILED;
    }
    if (fd != -1) {
      close(fd);
      fd = -1;
    }
    if (sem != SEM_FAILED) {
      sem_close(sem);
      sem = SEM_FAILED;
    }
  }

} naoBodyAccess;

NaoBody::NaoBody() : writingActuators(-1), fdCpuTemp(0) {}

NaoBody::~NaoBody() {
  if (fdCpuTemp)
    fclose(fdCpuTemp);
}

bool NaoBody::init() {
  return naoBodyAccess.init();
}

void NaoBody::shutdown() {
  naoBodyAccess.lbhData->state = shutdownState;
}

void NaoBody::cleanup() {
  naoBodyAccess.cleanup();
}

void NaoBody::setCrashed(int termSignal) {
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  naoBodyAccess.lbhData->state = BHState(termSignal);
}

bool NaoBody::wait() {
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  ASSERT(naoBodyAccess.sem != SEM_FAILED);
  do {
    if (sem_wait(naoBodyAccess.sem) == -1) {
      bool success = false;
      while (errno == 516) {
        if (sem_wait(naoBodyAccess.sem) == -1) {
          ASSERT(false);
          continue;
        } else {
          success = true;
          break;
        }
      }
      if (!success) {
        ASSERT(false);
        return false;
      }
    }
  } while (naoBodyAccess.lbhData->readingSensors == naoBodyAccess.lbhData->newestSensors);
  naoBodyAccess.lbhData->readingSensors = naoBodyAccess.lbhData->newestSensors;

  static bool shout = true;
  if (shout) {
    shout = false;
    printf("nao bridge is working\n");
  }

  return true;
}

const char* NaoBody::getName() const {
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  return naoBodyAccess.lbhData->robotName;
}

float* NaoBody::getSensors() {
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  return naoBodyAccess.lbhData->sensors[naoBodyAccess.lbhData->readingSensors];
}

const RoboCup::RoboCupGameControlData& NaoBody::getGameControlData() const {
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  return naoBodyAccess.lbhData->gameControlData[naoBodyAccess.lbhData->readingSensors];
}

void NaoBody::getTemperature(float& cpu, float& mb) {
  cpu = mb = 0;

#ifndef NDEBUG
  if (!fdCpuTemp) {
#ifdef ROBOT_V6
    fdCpuTemp = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
#else // ROBOT_V5
    fdCpuTemp = fopen("/proc/acpi/thermal_zone/THRM/temperature", "r");
#endif
    ASSERT(fdCpuTemp);
  }

  if (fdCpuTemp) {
#ifdef ROBOT_V6
    VERIFY(fseek(fdCpuTemp, 0, SEEK_SET) == 0);
#else // ROBOT_V5
    VERIFY(fseek(fdCpuTemp, 20, SEEK_SET) == 0);
#endif
    VERIFY(fscanf(fdCpuTemp, "%f", &cpu) == 1);
  }
#endif
}

bool NaoBody::getWlanStatus() {
  return access("/sys/class/net/wlan0", F_OK) == 0;
}

void NaoBody::openActuators(float*& actuators) {
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  ASSERT(writingActuators == -1);
  writingActuators = 0;
  if (writingActuators == naoBodyAccess.lbhData->newestActuators)
    ++writingActuators;
  if (writingActuators == naoBodyAccess.lbhData->readingActuators)
    if (++writingActuators == naoBodyAccess.lbhData->newestActuators)
      ++writingActuators;
  ASSERT(writingActuators != naoBodyAccess.lbhData->newestActuators);
  ASSERT(writingActuators != naoBodyAccess.lbhData->readingActuators);
  actuators = naoBodyAccess.lbhData->actuators[writingActuators];
}

void NaoBody::closeActuators() {
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  ASSERT(writingActuators >= 0);
  naoBodyAccess.lbhData->newestActuators = writingActuators;
  writingActuators = -1;
}

void NaoBody::setRobotInfo(uint8_t teamNumber,
                           uint8_t teamColour,
                           uint8_t playerNumber,
                           uint8_t playerFallen,
                           float poseX,
                           float poseY,
                           float poseTheta,
                           float ballAge,
                           float ballX,
                           float ballY) {
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  naoBodyAccess.lbhData->robotInfo[::teamNumber] = teamNumber;
  naoBodyAccess.lbhData->robotInfo[::teamColour] = teamColour;
  naoBodyAccess.lbhData->robotInfo[::playerNumber] = playerNumber;
  naoBodyAccess.lbhData->robotInfo[::playerFallen] = playerFallen;
  naoBodyAccess.lbhData->robotInfo[::playerPoseX] = poseX * 1000ll;
  naoBodyAccess.lbhData->robotInfo[::playerPoseY] = poseY * 1000ll;
  naoBodyAccess.lbhData->robotInfo[::playerPoseTheta] = poseTheta * 1000ll;
  naoBodyAccess.lbhData->robotInfo[::playerBallAge] = ballAge * 1000ll;
  naoBodyAccess.lbhData->robotInfo[::playerBallX] = ballX * 1000ll;
  naoBodyAccess.lbhData->robotInfo[::playerBallY] = ballY * 1000ll;
  naoBodyAccess.lbhData->startTime = Time::getSystemTimeBase() | 1; // make sure it's non zero
}
