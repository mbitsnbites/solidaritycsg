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

#include "../Vector3.h"
#include "../SampleSpace.h"
#include "../FileIO/Mesh.h"

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

    /// Triangle mesh. This mesh is filled out by the AppendSlicePair() method.
    Mesh mMesh;

  private:
    /// Process a single cube, and append the result to the mesh.
    void PorcessOneCube(Cube &aCube, Voxel aLevel);

    /// Voxel sample space definition.
    SampleSpace * mSampleSpace;
};

}

#endif // _POLYGONIZE_H_
