/*
  This file is part of SolidarityCSG.

  SolidarityCSG is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  SolidarityCSG is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with SolidarityCSG.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _OSTHREAD_H_
#define _OSTHREAD_H_

//------------------------------------------------------------------------------
// Description:
// This is a minimal, portable implementation of basic threading classes.
//
// They closely mimic the functionality and naming of the C++0x standard, and
// should be fully replaceable with the corresponding std:: variants, once they
// are widely supported by compilers.
//
// The Win32 variant uses the native Win32 API for implementing the thread
// classes, while for other systems, the POSIX threads API is used.
//------------------------------------------------------------------------------

#include "Platform.h"
#ifdef WIN32
 #include <windows.h>
#else
 #include <pthread.h>
#endif


namespace os {

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
      CloseHandle(mThread);
#else
      // FIXME
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

    /// This method is a wrapper function that is called from within the
    /// newly created thread (do not call).
    void _execute()
    {
      mFunction(mArg);
    }

  private:
    void (*mFunction)(void *); ///< Pointer to the function to be executed.
    void * mArg;               ///< Function argument for the thread function.
#ifdef WIN32
    HANDLE mThread;
    DWORD mThreadID;
#else
    pthread_t mThread;
#endif
};

}

#endif // _OSTHREAD_H_
