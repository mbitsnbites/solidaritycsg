/*
Copyright (c) 2010 Marcus Geelnard

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

    1. The origin of this software must not be misrepresented; you must not
    claim that you wrote the original software. If you use this software
    in a product, an acknowledgment in the product documentation would be
    appreciated but is not required.

    2. Altered source versions must be plainly marked as such, and must not be
    misrepresented as being the original software.

    3. This notice may not be removed or altered from any source
    distribution.
*/

#ifndef _TINYTHREAD_H_
#define _TINYTHREAD_H_

// Which platform are we on?
#if !defined(WIN32) && defined(_WIN32)
  #define WIN32
#endif

// Platform specific includes
#ifdef WIN32
  #include <windows.h>
#else
  #include <pthread.h>
  #include <signal.h>
#endif


namespace tinythread {

/// @mainpage TinyThread++ API Reference
///
/// @section intro_sec Introduction
/// TinyThread++ is a minimal, portable implementation of basic threading
/// classes for C++.
///
/// They closely mimic the functionality and naming of the C++0x standard, and
/// should be easily replaceable with the corresponding std:: variants.
///
/// @section port_sec Portability
/// The Win32 variant uses the native Win32 API for implementing the thread
/// classes, while for other systems, the POSIX threads API (pthread) is used.
///
/// @section class_sec Classes
/// There are three classes that are implemented: thread, mutex, and
/// condition_variable.
///
/// @section misc_sec Miscellaneous
/// The following additional functions are available: NumberOfCores().


/// Mutex class. This is a mutual exclusion object for synchronizing access to
/// shared memory areas for several threads.
class mutex {
  public:
    /// Constructor.
    mutex()
    {
#ifdef WIN32
      InitializeCriticalSection(&mMutex);
#else
      pthread_mutex_init(&mMutex, NULL);
#endif
    }

    /// Destructor.
    ~mutex()
    {
#ifdef WIN32
      DeleteCriticalSection(&mMutex);
#else
      pthread_mutex_destroy(&mMutex);
#endif
    }

    /// Lock the mutex.
    inline void lock()
    {
#ifdef WIN32
      EnterCriticalSection(&mMutex);
#else
      pthread_mutex_lock(&mMutex);
#endif
    }

    /// Unlock the mutex.
    inline void unlock()
    {
#ifdef WIN32
      LeaveCriticalSection(&mMutex);
#else
      pthread_mutex_unlock(&mMutex);
#endif
    }

  private:
#ifdef WIN32
    CRITICAL_SECTION mMutex;
#else
    pthread_mutex_t mMutex;
#endif

    friend class condition_variable;
};

/// Condition variable class. This is a signalling object for synchronizing the
/// execution flow for several threads.
class condition_variable {
  public:
    /// Constructor.
#ifdef WIN32
    condition_variable();
#else
    condition_variable()
    {
      pthread_cond_init(&mCondition, NULL);
    }
#endif

    /// Destructor.
#ifdef WIN32
    ~condition_variable();
#else
    ~condition_variable()
    {
      pthread_cond_destroy(&mCondition);
    }
#endif

    /// Wait for the condition.
#ifdef WIN32
    void wait(mutex &aMutex);
#else
    inline void wait(mutex &aMutex)
    {
      pthread_cond_wait(&mCondition, &aMutex.mMutex);
    }
#endif

    /// Notify one thread that is waiting for the condition.
#ifdef WIN32
    void notify_one();
#else
    inline void notify_one()
    {
      pthread_cond_signal(&mCondition);
    }
#endif

    /// Notify all threads that are waiting for the condition.
#ifdef WIN32
    void notify_all();
#else
    inline void notify_all()
    {
      pthread_cond_broadcast(&mCondition);
    }
#endif

  private:
#ifdef WIN32
    HANDLE mEvents[2];                  ///< Signal and broadcast event HANDLEs.
    unsigned int mWaitersCount;         ///< Count of the number of waiters.
    CRITICAL_SECTION mWaitersCountLock; ///< Serialize access to mWaitersCount.
#else
    pthread_cond_t mCondition;
#endif
};

/// Thread class.
class thread {
  public:
    /// Constructor. This version is not fully compatible with the standard C++
    /// thread class, but does its job.
    /// \param aFunction A function pointer to a function of type:
    ///                  void fun(void * arg)
    /// \param aArg Argument to the thread function.
    thread(void (*aFunction)(void *), void * aArg);

    /// Destructor.
    ~thread()
    {
#ifdef WIN32
      TerminateThread(mThread, 0);
      CloseHandle(mThread);
#else
      pthread_kill(mThread, SIGKILL);
#endif
    }

    /// Wait for the thread to finish (join execution flows).
    inline void join()
    {
#ifdef WIN32
      WaitForSingleObject(mThread, INFINITE);
#else
      pthread_join(mThread, NULL);
#endif
    }

  private:
#ifdef WIN32
    HANDLE mThread;
#else
    pthread_t mThread;
#endif
};


/// Determine the number of CPU cores in the system. This function is useful
/// for determining the optimal number of threads to use for a task.
int number_of_cores();

}

#endif // _TINYTHREAD_H_
