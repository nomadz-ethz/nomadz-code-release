/**
 * @file SystemCall.h
 *
 * Implementation of system calls and access to thread local storage.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <A href=mailto:brunn@sim.informatik.tu-darmstadt.de>Ronnie Brunn</A> and <A
 * href=mailto:risler@sim.informatik.tu-darmstadt.de>Max Risler</A>
 */

#pragma once

#include <cstdlib>

#define SYSTEMCALL_INCLUDED

/**
 * All process-local global variable declarations have to be preceeded
 * by this macro. Only variables of simple types can be defined, i.e.
 * no class instantiations are allowed.
 */
#define PROCESS_WIDE_STORAGE(type) __thread type*

/**
 * static class for system calls
 * @attention the implementation is system specific!
 */
class SystemCall {
public:
  enum Mode {
    physicalRobot,
    remoteRobot,
    simulatedRobot,
    logfileReplay,
    teamRobot,
  };

  /** returns the name of the local machine*/
  static const char* getHostName();

  /** returns the first ip address of the local machine*/
  static const char* getHostAddr();

  /** returns the current execution mode */
  static Mode getMode();

  /** waits a certain number of ms. */
  static void sleep(unsigned ms);

  /** Returns the load and the physical memory usage in percent */
  static void getLoad(float& mem, float load[3]);

  /** Allocate memory of given size with given alignment. */
  static void* alignedMalloc(size_t size, size_t alignment = 16);

  /** Free aligned memory.*/
  static void alignedFree(void* ptr);

  /**
   * Put a filename into play sound queue.
   * If you want to play Config/Sounds/bla.wav use play("bla.wav");
   * @param name The filename of the sound file.
   * @return The amound of files in play sound queue.
   */
  static int playSound(const char* name);
};
