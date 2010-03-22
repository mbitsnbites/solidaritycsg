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

#ifndef _POLYGONIZE_H_
#define _POLYGONIZE_H_

#include "../SampleSpace.h"
#include "../FileIO/Mesh.h"
#include "TriBuf.h"

namespace csg {

class Cube;

/// Polygonize class.
class Polygonize {
  public:
    /// Constructor.
    Polygonize()
    {
      mSampleSpace = 0;
    }

    /// Define the voxel sample space boundaries and resolution.
    inline void SetSampleSpace(SampleSpace * aSampleSpace)
    {
      mSampleSpace = aSampleSpace;
    }

    /// Calculate the polygons for a pair of slices of the voxel volume.
    void AppendSlicePair(Voxel * aSlice1, Voxel * aSlice2, int aZ1);

    /// Export the state of the polygonizer to a mesh object.
    void ToMesh(Mesh &aMesh);

    /// Return the total triangle count.
    inline int Count()
    {
      return mTriBuf.Count();
    }

  private:
    /// Voxel sample space definition.
    SampleSpace * mSampleSpace;

    /// Triangle buffer.
    TriBuf mTriBuf;
};

}

#endif // _POLYGONIZE_H_
