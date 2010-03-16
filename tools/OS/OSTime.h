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

#ifndef _OSTIME_H_
#define _OSTIME_H_

#include "Platform.h"
#include <list>

namespace os {

class Timer {
  private:
    std::list<double> mStack;
#ifdef WIN32
    __int64 mTimeFreq;
    __int64 mTimeStart;
#else
    long long mTimeStart;
#endif

  public:
    /// Constructor
    Timer();

    /// Get current time.
    double Get();

    /// Push current time (start measuring).
    void Push();

    /// Pop delta time since last push.
    double PopDelta();
};

}

#endif // _OSTIME_H_
