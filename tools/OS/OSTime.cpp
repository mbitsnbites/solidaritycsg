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

#include "OSTime.h"

#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

using namespace std;

namespace os {

/// Constructor
Timer::Timer() {
#ifdef WIN32
  if (QueryPerformanceFrequency((LARGE_INTEGER*)&mTimeFreq))
    QueryPerformanceCounter((LARGE_INTEGER*)&mTimeStart);
  else
    mTimeFreq = 0;
#else
  struct timeval tv;
  gettimeofday(&tv, 0);
  mTimeStart = (long long)tv.tv_sec * (long long)1000000 + (long long)tv.tv_usec;
#endif
}

/// Get current time.
double Timer::Get() {
#ifdef WIN32
  __int64 t;
  QueryPerformanceCounter((LARGE_INTEGER*)&t);
  return double(t - mTimeStart) / double(mTimeFreq);
#else
  struct timeval tv;
  gettimeofday(&tv, 0);
  long long t = (long long)tv.tv_sec * (long long)1000000 + (long long)tv.tv_usec;
  return (1e-6) * double(t - mTimeStart);
#endif
}

/// Push current time (start measuring).
void Timer::Push() {
  mStack.push_back(Get());
}

/// Pop delta time since last push.
double Timer::PopDelta() {
  double delta = Get() - mStack.back();
  mStack.pop_back();
  return delta;
}

}  // namespace os
