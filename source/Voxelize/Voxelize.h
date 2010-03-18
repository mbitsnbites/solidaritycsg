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

#ifndef _VOXELIZE_H_
#define _VOXELIZE_H_

#include "../Vector3.h"
#include "../BoundingBox.h"
#include "../SampleSpace.h"

namespace csg {

/// Voxelize class.
class Voxelize {
  public:
    /// Constructor.
    Voxelize()
    {
      mSampleSpace = 0;
    }

    /// Destructor.
    virtual ~Voxelize() {}

    /// Return the minimum enclosing axis aligned bounding box for this shape.
    inline void GetBoundingBox(BoundingBox &aAABB)
    {
      aAABB = mAABB;
    }

    /// Define the voxel sample space boundaries and resolution.
    inline void SetSampleSpace(SampleSpace * aSampleSpace)
    {
      mSampleSpace = aSampleSpace;
    }

    /// Calculate a single slice of the voxel volume. The slice must be
    /// allocated by the caller, and hold DivX * DivY voxels. The function
    /// returns false if all elements were cosidered "outside" (i.e. the slice
    /// is empty).
    virtual bool CalculateSlice(Voxel * aSlice, int aZ) = 0;

  protected:
    SampleSpace * mSampleSpace; ///< Voxel sample space definition.
    BoundingBox mAABB;          ///< Shape bouinding box.
};

}

#endif // _VOXELIZE_H_
