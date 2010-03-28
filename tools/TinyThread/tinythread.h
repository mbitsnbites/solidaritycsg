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

// Generic includes
#include <ostream>

// TinyThread++ version
#define TINYTHREAD_VERSION_MAJOR 0
#define TINYTHREAD_VERSION_MINOR 4


namespace tthread {

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
/// There are four classes that are implemented: thread, mutex, lock_guard and
/// condition_variable.
///
/// @section misc_sec Miscellaneous
/// The following additional functions are available: number_of_processors().

/// Mutex class.
/// This is a mutual exclusion object for synchronizing access to shared
/// memory areas for several threads.
/// @note Unlike the C++0x \c mutex class (which is strictly non-recursive),
/// this class may or may not be recursive, depending on the underlying system
/// implementation.
class mutex {
  public:
    /// Constructor.
    mutex()
    {
#ifdef WIN32
      InitializeCriticalSection(&mHandle);
#else
      pthread_mutex_init(&mHandle, NULL);
#endif
    }

    /// Destructor.
    ~mutex()
    {
#ifdef WIN32
      DeleteCriticalSection(&mHandle);
#else
      pthread_mutex_destroy(&mHandle);
#endif
    }

    /// Lock the mutex.
    /// The method will block the calling thread until a lock on the mutex can
    /// be obtained. The mutex remains locked until \c unlock() is called.
    /// @see lock_guard
    inline void lock()
    {
#ifdef WIN32
      EnterCriticalSection(&mHandle);
#else
      pthread_mutex_lock(&mHandle);
#endif
    }

    /// Try to lock the mutex.
    /// The method will try to lock the mutex. If it fails, the function will
    /// return immediately (non-blocking).
    /// @return \c true if the lock was acquired, or \c false if the lock could
    /// not be acquired.
    inline bool try_lock()
    {
#ifdef WIN32
      return TryEnterCriticalSection(&mHandle) ? true : false;
#else
      return (pthread_mutex_trylock(&mHandle) == 0) ? true : false;
#endif
    }

    /// Unlock the mutex.
    /// If any threads are waiting for the lock on this mutex, one of them will
    /// be unblocked.
    inline void unlock()
    {
#ifdef WIN32
      LeaveCriticalSection(&mHandle);
#else
      pthread_mutex_unlock(&mHandle);
#endif
    }

  private:
#ifdef WIN32
    CRITICAL_SECTION mHandle;
#else
    pthread_mutex_t mHandle;
#endif

    friend class condition_variable;
};

/// Lock guard class.
/// The constructor locks the mutex, and the destructor unlocks the mutex, so
/// the mutex will automatically be unlocked when the lock guard goes out of
/// scope. Example usage:
/// @code
/// mutex m;
/// int counter;
///
/// void increment()
/// {
///   lock_guard<mutex> guard(m);
///   ++ counter;
/// }
/// @endcode

template <class T>
class lock_guard {
  public:
    typedef T mutex_type;

    lock_guard() : mMutex(0) {}

    /// The constructor locks the mutex.
    explicit lock_guard(mutex_type &aMutex)
    {
      mMutex = &aMutex;
      mMutex->lock();
    }

    /// The destructor unlocks the mutex.
    ~lock_guard()
    {
      if(mMutex)
        mMutex->unlock();
    }

  private:
    mutex_type * mMutex;
};

/// Condition variable class.
/// This is a signalling object for synchronizing the execution flow for
/// several threads. Example usage:
/// @code
/// // Shared data and associated mutex and condition variable objects
/// int count;
/// mutex m;
/// condition_variable cond;
///
/// // Wait for the counter to reach a certain number
/// void wait_counter(int targetCount)
/// {
///   lock_guard<mutex> guard(m);
///   while(count < targetCount)
///     cond.wait(m);
/// }
///
/// // Increment the counter, and notify waiting threads
/// void increment()
/// {
///   lock_guard<mutex> guard(m);
///   ++ count;
///   cond.notify_all();
/// }
/// @endcode
class condition_variable {
  public:
    /// Constructor.
#ifdef WIN32
    condition_variable();
#else
    condition_variable()
    {
      pthread_cond_init(&mHandle, NULL);
    }
#endif

    /// Destructor.
#ifdef WIN32
    ~condition_variable();
#else
    ~condition_variable()
    {
      pthread_cond_destroy(&mHandle);
    }
#endif

    /// Wait for the condition.
    /// The function will block the calling thread until the condition variable
    /// is woken by \c notify_one(), \c notify_all() or a spurious wake up.
    /// @param[in] aMutex A mutex that will be unlocked when the wait operation
    ///   starts, an locked again as soon as the wait operation is finished.
#ifdef WIN32
    void wait(mutex &aMutex);
#else
    inline void wait(mutex &aMutex)
    {
      pthread_cond_wait(&mHandle, &aMutex.mHandle);
    }
#endif

    /// Notify one thread that is waiting for the condition.
    /// If at least one thread is blocked waiting for this condition variable,
    /// one will be woken up.
    /// @note Only threads that started waiting prior to this call will be
    /// woken up.
#ifdef WIN32
    void notify_one();
#else
    inline void notify_one()
    {
      pthread_cond_signal(&mHandle);
    }
#endif

    /// Notify all threads that are waiting for the condition.
    /// All threads that are blocked waiting for this condition variable will
    /// be woken up.
    /// @note Only threads that started waiting prior to this call will be
    /// woken up.
#ifdef WIN32
    void notify_all();
#else
    inline void notify_all()
    {
      pthread_cond_broadcast(&mHandle);
    }
#endif

  private:
#ifdef WIN32
    HANDLE mEvents[2];                  ///< Signal and broadcast event HANDLEs.
    unsigned int mWaitersCount;         ///< Count of the number of waiters.
    CRITICAL_SECTION mWaitersCountLock; ///< Serialize access to mWaitersCount.
#else
    pthread_cond_t mHandle;
#endif
};


/// Thread ID.
/// The thread ID is a unique identifier for each thread.
class id {
  public:
    /// Default constructor.
    /// The default constructed ID is that of thread without a thread of
    /// execution.
    id() : mId(0) {};

    id(unsigned long int aId) : mId(aId) {};

    id & operator=(const id &aId)
    {
      mId = aId.mId;
      return *this;
    }

    bool operator==(const id &aId)
    {
      return (aId.mId == mId);
    }

    bool operator!=(const id &aId)
    {
      return (aId.mId != mId);
    }

  private:
    unsigned long int mId;

  friend std::ostream& operator <<(std::ostream &os, const id &obj);
};


/// Thread class.
class thread {
  public:
#ifdef WIN32
    typedef HANDLE native_handle_type;
#else
    typedef pthread_t native_handle_type;
#endif

    /// Default constructor.
    /// Construct a \c thread object without an associated thread of execution
    /// (i.e. non-joinable).
    thread() : mHandle(0), mNotAThread(true)
#ifdef WIN32
    , mWi32ThreadID(0)
#endif
    {}

    /// Thread starting constructor.
    /// Construct a \c thread object with a new thread of execution.
    /// @param[in] aFunction A function pointer to a function of type:
    ///          <tt>void fun(void * arg)</tt>
    /// @param[in] aArg Argument to the thread function.
    /// @note This constructor is not fully compatible with the standard C++
    /// thread class. It is more similar to the pthread_create() (POSIX) and
    /// CreateThread() (Windows) functions.
    thread(void (*aFunction)(void *), void * aArg);

    /// Destructor.
    /// @note If the thread is joinable upon destruction, \c std::terminate()
    /// will be called, which terminates the process. It is always wise to do
    /// \c join() before deleting a thread object.
    ~thread();

    /// Wait for the thread to finish (join execution flows).
    void join();

    /// Check if the thread is joinable.
    /// A thread object is joinable if it has an associated thread of execution.
    bool joinable() const;

    /// Return the thread ID of a thread object.
    id get_id() const;

    /// Get the native handle for this thread.
    /// @note Under Windows, this is a \c HANDLE, and under POSIX systems, this
    /// is a \c pthread_t.
    inline native_handle_type native_handle()
    {
      return mHandle;
    }

  private:
    native_handle_type mHandle; ///< Thread handle.
    mutable mutex mDataMutex;   ///< Serializer for access to the thread private data.
    bool mNotAThread;           ///< True if this object is not a thread of execution.
#ifdef WIN32
    DWORD mWi32ThreadID;        ///< Unique thread ID (filled out by CreateThread).
#endif

    // The internal thread wrapper function needs access to the internal thread
    // data.
    friend void _thread_wrapper(void * aArg);
};


namespace this_thread {
  /// Return the thread ID of the calling thread.
  id get_id();
}

/// Determine the number of processors (CPU cores) in the system. This function
/// is useful for determining the optimal number of threads to use for a task.
int number_of_processors();

}

#endif // _TINYTHREAD_H_
