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
#include "tinythread.h"
#ifndef WIN32
#include <unistd.h>
#include <map>
#endif


namespace tthread {

//------------------------------------------------------------------------------
// id << operator
//------------------------------------------------------------------------------

std::ostream& operator <<(std::ostream &os, const tthread::id &obj)
{
  os << obj.mId;
  return os;
}


//------------------------------------------------------------------------------
// condition_variable
//------------------------------------------------------------------------------
// NOTE 1: The Win32 implementation of the condition_variable class is based on
// the corresponding implementation in GLFW, which in turn is based on a
// description by Douglas C. Schmidt and Irfan Pyarali:
// http://www.cs.wustl.edu/~schmidt/win32-cv-1.html
//
// NOTE 2: Windows Vista actually has native support for condition variables
// (InitializeConditionVariable, WakeConditionVariable, etc), but we want to
// be portable with pre-Vista Windows version, so TinyThread++ does not use
// Vista condition variables.
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
  LeaveCriticalSection(&aMutex.mHandle);

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
  EnterCriticalSection(&aMutex.mHandle);
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


#ifndef WIN32

//------------------------------------------------------------------------------
// POSIX pthread_t to unique tinythread::id mapping logic.
// Note: Here we use a global thread safe std:map to convert instances of
// pthread_t to small thread identifier numbers (unique within one process).
// This method should be portable across different POSIX implementations.
//------------------------------------------------------------------------------

// pthread_t -> ID map variables
static mutex _gIdMapLock;
static std::map<pthread_t, unsigned long int> _gIdMap;
static unsigned long int _gIdCount(1);

// This function converts a pthread_t "handle" to a unique thread id.
static id _pthread_t_to_ID(const pthread_t &aHandle)
{
  lock_guard<mutex> guard(_gIdMapLock);
  if(_gIdMap.find(aHandle) == _gIdMap.end())
    _gIdMap[aHandle] = _gIdCount ++;
  return id(_gIdMap[aHandle]);
}

#endif // !WIN32



//------------------------------------------------------------------------------
// thread
//------------------------------------------------------------------------------

/// Information to pass to the new thread (what to run).
struct _ThreadStartInfo {
  void (*mFunction)(void *); ///< Pointer to the function to be executed.
  void * mArg;               ///< Function argument for the thread function.
  thread * mThread;          ///< Pointer to the thread object.
};

// Thread wrapper function.
void _thread_wrapper(void * aArg)
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

  // The thread is no longer executing
  lock_guard<mutex> guard(ti->mThread->mDataMutex);
  ti->mThread->mNotAThread = true;

  // The thread is responsible for freeing the startup information
  delete ti;
}

// System specific thread wrapper function.
#ifdef WIN32
DWORD WINAPI _sys_thread_wrapper(LPVOID aArg)
{
  _thread_wrapper((void *) aArg);
  return 0;
}
#else
void * _sys_thread_wrapper(void * aArg)
{
  _thread_wrapper(aArg);
  return 0;
}
#endif

thread::thread(void (*aFunction)(void *), void * aArg)
{
  // Serialize access to this thread structure
  lock_guard<mutex> guard(mDataMutex);

  // Fill out the thread startup information (passed to the thread wrapper,
  // which will eventually free it)
  _ThreadStartInfo * ti = new _ThreadStartInfo;
  ti->mFunction = aFunction;
  ti->mArg = aArg;
  ti->mThread = this;

  // The thread is now alive
  mNotAThread = false;

#ifdef WIN32
  // Create the thread
  mHandle = CreateThread(0, 0, _sys_thread_wrapper, (LPVOID) ti, 0, &mWi32ThreadID);
  if(!mHandle)
    mNotAThread = true;
#else
  // Create the thread
  if(pthread_create(&mHandle, NULL, _sys_thread_wrapper, (void *) ti) != 0)
  {
    mHandle = 0;
    mNotAThread = true;
  }
#endif
}

thread::~thread()
{
  if(joinable())
    std::terminate();
}

void thread::join()
{
  if(joinable())
  {
#ifdef WIN32
    WaitForSingleObject(mHandle, INFINITE);
#else
    pthread_join(mHandle, NULL);
#endif
  }
}

bool thread::joinable() const
{
  mDataMutex.lock();
  bool result = !mNotAThread;
  mDataMutex.unlock();
  return result;
}

id thread::get_id() const
{
  if(!joinable())
    return id();
#ifdef WIN32
  return id((unsigned long int) mWi32ThreadID);
#else
  return _pthread_t_to_ID(mHandle);
#endif
}


//------------------------------------------------------------------------------
// this_thread
//------------------------------------------------------------------------------

id this_thread::get_id()
{
#ifdef WIN32
  return id((unsigned long int) GetCurrentThreadId());
#else
  return _pthread_t_to_ID(pthread_self());
#endif
}


//------------------------------------------------------------------------------
// Misc. functions
//------------------------------------------------------------------------------

int number_of_processors()
{
#if defined(WIN32)
  SYSTEM_INFO si;
  GetSystemInfo(&si);
  return (int) si.dwNumberOfProcessors;
#elif defined(_SC_NPROCESSORS_ONLN)
  return (int) sysconf(_SC_NPROCESSORS_ONLN);
#elif defined(_SC_NPROC_ONLN)
  return (int) sysconf(_SC_NPROC_ONLN);
#else
  return 1;
#endif
}

}
