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

#include <exception>
#include <unistd.h>
#include "tinythread.h"

namespace tinythread {

//------------------------------------------------------------------------------
// condition_variable
//------------------------------------------------------------------------------
// NOTE: The Win32 implementation of the condition_variable class is based on
// the corresponding implementation in GLFW, which in turn is based on a
// description by Douglas C. Schmidt and Irfan Pyarali:
// http://www.cs.wustl.edu/~schmidt/win32-cv-1.html
//------------------------------------------------------------------------------

#ifdef WIN32
  #define _CONDITION_EVENT_ONE 0
  #define _CONDITION_EVENT_ALL 1
#endif

#ifdef WIN32
condition_variable::condition_variable()
{
  mWaitersCount = 0;
  mEvents[_CONDITION_EVENT_ONE] = CreateEvent(NULL, FALSE, FALSE, NULL);
  mEvents[_CONDITION_EVENT_ALL] = CreateEvent(NULL, TRUE, FALSE, NULL);
  InitializeCriticalSection(&mWaitersCountLock);
}
#endif

#ifdef WIN32
condition_variable::~condition_variable()
{
  CloseHandle(mEvents[_CONDITION_EVENT_ONE]);
  CloseHandle(mEvents[_CONDITION_EVENT_ALL]);
  DeleteCriticalSection(&mWaitersCountLock);
}
#endif

#ifdef WIN32
void condition_variable::wait(mutex &aMutex)
{
  // Increment number of waiters
  EnterCriticalSection(&mWaitersCountLock);
  ++ mWaitersCount;
  LeaveCriticalSection(&mWaitersCountLock);

  // It's ok to release the mutex here since Win32 manual-reset events maintain
  // state when used with SetEvent()
  LeaveCriticalSection(&aMutex.mMutex);

  // Wait for either event to become signaled due to notify_one() or
  // notify_all() being called
  int result = WaitForMultipleObjects(2, mEvents, FALSE, INFINITE);

  // Check if we are the last waiter
  EnterCriticalSection(&mWaitersCountLock);
  -- mWaitersCount;
  bool lastWaiter = (result == (WAIT_OBJECT_0 + _CONDITION_EVENT_ALL)) &&
                    (mWaitersCount == 0);
  LeaveCriticalSection(&mWaitersCountLock);

  // If we are the last waiter to be notified to stop waiting, reset the event
  if(lastWaiter)
    ResetEvent(mEvents[_CONDITION_EVENT_ALL]);

  // Reacquire the mutex
  EnterCriticalSection(&aMutex.mMutex);
}
#endif

#ifdef WIN32
void condition_variable::notify_one()
{
  // Are there any waiters?
  EnterCriticalSection(&mWaitersCountLock);
  bool haveWaiters = (mWaitersCount > 0);
  LeaveCriticalSection(&mWaitersCountLock);

  // If we have any waiting threads, send them a signal
  if(haveWaiters)
    SetEvent(mEvents[_CONDITION_EVENT_ONE]);
}
#endif

#ifdef WIN32
void condition_variable::notify_all()
{
  // Are there any waiters?
  EnterCriticalSection(&mWaitersCountLock);
  bool haveWaiters = (mWaitersCount > 0);
  LeaveCriticalSection(&mWaitersCountLock);

  // If we have any waiting threads, send them a signal
  if(haveWaiters)
    SetEvent(mEvents[_CONDITION_EVENT_ALL]);
}
#endif

//------------------------------------------------------------------------------
// thread
//------------------------------------------------------------------------------

/// Information to pass to the new thread (what to run).
struct _ThreadStartInfo {
  void (*mFunction)(void *); ///< Pointer to the function to be executed.
  void * mArg;               ///< Function argument for the thread function.
};

// Thread wrapper function.
#ifdef WIN32
DWORD WINAPI _threadWrapper(LPVOID aArg)
#else
void * _threadWrapper(void * aArg)
#endif
{
  // Get thread startup information
  _ThreadStartInfo * ti = (_ThreadStartInfo *) aArg;

  try
  {
    // Call the actual client thread function
    ti->mFunction(ti->mArg);
  }
  catch(...)
  {
    // Uncaught exceptions will terminate the application (default behavior
    // according to the C++0x draft)
    std::terminate();
  }

  // The thread is responsible for freeing the startup information
  delete ti;

  return 0;
}

thread::thread(void (*aFunction)(void *), void * aArg)
{
  // Fill out the thread startup information (passed to the thread wrapper,
  // which will eventually free it)
  _ThreadStartInfo * ti = new _ThreadStartInfo;
  ti->mFunction = aFunction;
  ti->mArg = aArg;

#ifdef WIN32
  // Create the thread
  mThread = CreateThread(0, 0, _threadWrapper, (LPVOID) ti, 0, 0);
#else
  // Create the thread
  pthread_create(&mThread, NULL, _threadWrapper, (void *) ti);
#endif
}


//------------------------------------------------------------------------------
// Misc. functions
//------------------------------------------------------------------------------

int number_of_cores()
{
#ifdef WIN32
  SYSTEM_INFO si;
  GetSystemInfo(&si);
  return (int) si.dwNumberOfProcessors;
#else
  #if defined(_SC_NPROCESSORS_ONLN)
    return (int) sysconf(_SC_NPROCESSORS_ONLN);
  #elif defined(_SC_NPROC_ONLN)
    return (int) sysconf(_SC_NPROC_ONLN);
  #else
    return 1;
  #endif
#endif
}

}
