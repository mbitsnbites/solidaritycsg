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

#ifndef _ARRAY_H_
#define _ARRAY_H_

namespace csg {

/// Class for fixed sized (static) arrays. This is a simple wrapper class for
/// easy memory management of arrays that are allocated on the heap (e.g. to
/// avoid memory leaks when an exception occurs).
template <class T>
class Array {
public:
  /// Constructor.
  Array(unsigned int aSize) {
    mData = new T[aSize];
    mSize = aSize;
  }

  /// Destructor.
  ~Array() {
    delete[] mData;
  }

  /// Return the size of the array.
  inline unsigned int size() {
    return mSize;
  }

  /// Indexing operator.
  inline T& operator[](unsigned int aIdx) const {
    return mData[aIdx];
  }

private:
  T* mData;            ///< Actual data array.
  unsigned int mSize;  ///< Size of the array.
};

}  // namespace csg

#endif  // _ARRAY_H_
