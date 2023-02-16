/**
 * @class RefZMP.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
 */
#pragma once

#include "Tools/DortmundWalkingEngine/StepData.h"

#ifndef WALKING_SIMULATOR
#include "Core/Streams/AutoStreamable.h"
#include "Core/Debugging/Watch.h"
#else
#include "bhumanstub.h"
#include "Watch.h"
#endif

/** Maximum number of possible foot positions in buffer */
#define MAX_ZMP 300

/**
 * @class RefZMP
 * Representation to transfer the reference ZMP.
 */
STREAMABLE_DECLARE_LEGACY(RefZMP)
class RefZMP : public RefZMPBaseWrapper {
public:
  int numOfZMP;     /**< Number of ZMP point stored in the buffer */
  int numOfZMP_RCS; /**< Number of ZMP point in RCS stored in the buffer */
  bool running;     /**< Is the controller running? */

  void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(numOfZMP)
    STREAM(numOfZMP_RCS)
    STREAM(running)
    STREAM(zmp[0].x)
    STREAM(zmp[0].y)
    STREAM(zmp_RCS[0].x)
    STREAM(zmp_RCS[0].y)
    STREAM_REGISTER_FINISH;
  };

  /** Constructor */
  RefZMP() : running(false) {
    numOfZMP = 0;
    numOfZMP_RCS = 0;
  };

  /** Desctructor */
  ~RefZMP(){};

  /** Initialize the data. */
  void init() {
    numOfZMP = 0;
    numOfZMP_RCS = 0;
  }

  /**
   * Adds a reference ZMP to the buffer.
   * \param newzmp Reference ZMP to add.
   */
  void addZMP(ZMP newzmp) {
    if (numOfZMP > MAX_ZMP) {
      printf("addZMP(): Overflow");
      return;
    }
    zmp[numOfZMP] = newzmp;
    numOfZMP++;
  }

  /**
   * Adds a reference ZMP in RCS to the buffer.
   * \param newzmp_RCS Reference ZMP in RCS to add.
   */
  void addZMP_RCS(ZMP newzmp_RCS) {
    if (numOfZMP_RCS > MAX_ZMP) {
      printf("addZMP(): Overflow");
      return;
    }
    zmp_RCS[numOfZMP_RCS] = newzmp_RCS;
    numOfZMP_RCS++;
  }

  ZMP getZMP(int i) const {
    if (i > MAX_ZMP) {
      printf("getZMP(): Index out of range");
      return zmp[0];
    }
    return zmp[i];
  }

  ZMP getZMP_RCS(int i) const {
    if (i > MAX_ZMP) {
      printf("getZMP(): Index out of range");
      return zmp_RCS[0];
    }
    return zmp_RCS[i];
  }

private:
  ZMP zmp[MAX_ZMP];     /**< Buffer with reference ZMPs. */
  ZMP zmp_RCS[MAX_ZMP]; /**< Buffer with reference ZMPs in RCS. */
};
