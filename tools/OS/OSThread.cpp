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

#include <stdexcept>
#include "OSThread.h"

using namespace std;

namespace os {

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
  mEvents[0] = CreateEvent(NULL, FALSE, FALSE, NULL);
  mEvents[1] = CreateEvent(NULL, TRUE, FALSE, NULL);
  InitializeCriticalSection(&mWaitersCountLock);
}
#endif

#ifdef WIN32
condition_variable::~condition_variable()
{
  CloseHandle(mEvents[0]);
  CloseHandle(mEvents[1]);
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

  // Some thread called notify_all()
  if(lastWaiter)
  {
    // We're the last waiter to be notified or to stop waiting, so reset the
    // manual event
    ResetEvent(mEvents[_CONDITION_EVENT_ALL]);
  }

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
  _ThreadStartInfo * ti = (_ThreadStartInfo *) aArg;
  ti->mFunction(ti->mArg);
  delete ti;
  return 0;
}

thread::thread(void (*aFunction)(void *), void * aArg)
{
  // Fill out the thread startup information (passed to the thread wrapper)
  _ThreadStartInfo * ti = new _ThreadStartInfo;
  ti->mFunction = aFunction;
  ti->mArg = aArg;

#ifdef WIN32
  // Create the thread
  mThread = CreateThread(
              0,
              0,
              _threadWrapper,
              (LPVOID) ti,
              0,
              &mThreadID
            );

  if(!mThread)
    throw runtime_error("Unable to create thread.");
#else
  // Create the thread
  pthread_create(&mThread, NULL, _threadWrapper, (void *) ti);
#endif
}


//------------------------------------------------------------------------------
// Misc. functions
//------------------------------------------------------------------------------

int NumberOfCores()
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
    return 2;
  #endif
#endif
}

}
