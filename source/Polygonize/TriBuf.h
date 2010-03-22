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

#ifndef _TRIBUF_H_
#define _TRIBUF_H_

#include <list>
#include <vector>

#include "../Vector3.h"
#include "../FileIO/Mesh.h"

namespace csg {

/// Triangle buffer bin class. A bin object is a single chunk of data, part of
/// the total data buffer in a TriBuf object.
class TriBufBin {
  public:
    /// Constructor.
    TriBufBin(int aCapacity)
    {
      mCoords.resize(aCapacity * 3);
      mCapacity = aCapacity;
      mCount = 0;
    }

    /// Tells if this bin is full.
    inline bool IsFull()
    {
      return mCount >= mCapacity;
    }

  private:
    /// Triangle buffer holder.
    std::vector<Vector3> mCoords;

    /// Count of triangles in this buffer bin.
    unsigned int mCount;

    /// Capacity of triangles in this buffer bin.
    unsigned int mCapacity;

  friend class TriBuf;
};

/// Triangle buffer class.
class TriBuf {
  public:
    /// Constructor.
    TriBuf()
    {
      mCurrent = 0;
      mCount = 0;
    }

    /// Destructor.
    ~TriBuf();

    /// Append another triangle.
    void Append(Vector3 &aPoint1, Vector3 &aPoint2, Vector3 &aPoint3);

    /// Dump the triangle buffer to a mesh object. This will clear the
    /// triangle buffer (to save memory).
    void ToMesh(Mesh &aMesh);

    /// Return the total triangle count.
    inline int Count()
    {
      return mCount;
    }

  private:
    /// List of buffers.
    std::list<TriBufBin *> mBufList;

    /// Currrently active buffer item.
    TriBufBin * mCurrent;

    /// Total triangle count.
    int mCount;
};

}

#endif // _TRIBUF_H_
