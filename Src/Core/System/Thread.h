/**
 * @file Thread.h
 *
 * Declaration of a template class for threads and some
 * other classes for synchronization. Based on code
 * from B-Smart (real author unknown).
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Bernd.Gersdorf@dfki.de">Bernd Gersdorf</a>
 */

#pragma once

#include <atomic>
#include <thread>
#include <mutex>

#include <pthread.h>
#include <unistd.h>
#include <iostream>

#include "BHAssert.h"
#include "Semaphore.h"

#define THREAD_INCLUDED

/**
 * A class encapsulating a thread
 */
template <class T> class Thread {
private:
  Semaphore terminated;                /**< Has the thread terminated? */
  std::unique_ptr<std::thread> handle; /**< The thread handle */
  int priority;                        /**< The priority of the thread. */
  std::atomic_bool running;            /**< A flag which indicates the state of the thread */
  void (T::*function)();               /**< The address of the main function of the thread. */
  T* object;                           /**< A pointer to the object that is provided to the main function. */

  /**
   * The function is called when the thread is started.
   * It calls the main function of the thread as a member function of
   * an object.
   * @param p A pointer to the thread object.
   */
  static void* threadStart(Thread<T>* p) {
    ((p->object)->*(p->function))();
    p->running = false;
    return 0;
  }

public:
  /**
   * Default constructor.
   */
  Thread() : handle(nullptr), running(false) { setPriority(0); }

  /**
   * Destructor.
   * Stops the thread, if it is still running.
   */
  virtual ~Thread() { stop(); }

  /**
   * The function starts a member function as a new thread.
   * @param o The object the member function operates on.
   * @param f The member function.
   */
  void start(T* o, void (T::*f)()) {
    if (running)
      stop();
    function = f;
    object = o;
    running = true;
    handle = std::make_unique<std::thread>((void* (*)(void*)) & Thread<T>::threadStart, this);
    setPriority(priority);
  }

  /**
   * The function stops the thread.
   * It first signals its end by setting running to false. If the thread
   * does not terminate by itself, it will be killed after one second.
   */
  void stop() {
    running = false;
    if (handle) {
      VERIFY(handle->joinable());
      handle->join();
      handle = 0;
    }
  }

  /**
   * The function announces that the thread shall terminate.
   * It will not try to kill the thread.
   */
  virtual void announceStop() { running = false; }

  /**
   * The function sets the priority of the thread.
   * @param prio Priority relative to "normal" priority.
   */
  void setPriority(int prio) {
    ASSERT(prio == 0 || (prio > 0 && prio <= sched_get_priority_max(SCHED_FIFO)));
    priority = prio;
    if (handle) {
      sched_param param;
      param.sched_priority = priority;
      // FIXME: handle this properly, not just output an ERROR
      if (pthread_setschedparam(handle->native_handle(), priority == 0 ? SCHED_OTHER : SCHED_FIFO, &param)) {
        std::cerr << "ERROR: Failed to set thread real-time priority to " << priority << std::endl;
      }
    }
  }

  /**
   * The function determines whether the thread should still be running.
   * @return Should it continue?
   */
  bool isRunning() const { return running; }

  /**
   * The function returns the thread id.
   * @return The thread id. Only valid after the thread was started.
   */
  size_t getId() const {
    if (handle) {
      return std::hash<std::thread::id>{}(handle->get_id());
    } else {
      return 0;
    }
  };

  /**
   * The function returns the id of the calling thread.
   * @return The id of the calling thread.
   */
  static size_t getCurrentId() { return std::hash<std::thread::id>{}(std::this_thread::get_id()); }

  /**
   * Causes the calling thread to relinquish the CPU.
   */
  static void yield() { sched_yield(); }
};

/**
 * No naming of threads.
 * @param name The new name of the thread.
 */
#define NAME_THREAD(name) ((void)0)

/**
 * A class encapsulating a mutex lock.
 */
using SyncObject = std::mutex;

/**
 * The class provides a handy interface to using SyncObjects.
 */
class Sync {
private:
  SyncObject& syncObject; /**< A reference to a sync object. */
  std::unique_lock<SyncObject> lock;

public:
  /**
   * Constructor.
   * @param s A reference to a sync object representing a critical
   *          section. The section is entered.
   */
  Sync(SyncObject& s) : syncObject(s), lock(syncObject) {}

  /**
   * Destructor.
   * The critical section is left.
   */
  ~Sync() { lock.unlock(); }
};

/**
 * The macro places a SyncObject as member variable into a class.
 * This is the precondition for using the macro SYNC.
 */
#define DECLARE_SYNC mutable SyncObject _syncObject

/**
 * The macro SYNC ensures that the access to member variables is synchronized.
 * So only one thread can enter a SYNC block for this object at the same time.
 * The SYNC is automatically released at the end of the current code block.
 * Never nest SYNC blocks, because this will result in a deadlock!
 */
#define SYNC Sync _sync(_syncObject)

/**
 * The macro SYNC_WITH ensures that the access to the member variables of an
 * object is synchronized. So only one thread can enter a SYNC block for the
 * object at the same time. The SYNC is automatically released at the end of
 * the current code block. Never nest SYNC blocks, because this will result
 * in a deadlock!
 */
#define SYNC_WITH(obj) Sync _sync((obj)._syncObject)
