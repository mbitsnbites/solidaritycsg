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

#ifndef _SAMPLESPACE_H_
#define _SAMPLESPACE_H_

#include "Vector3.h"
#include "BoundingBox.h"

namespace csg {

/// The Voxel primitive is an 8 bit signed integer.
typedef signed char Voxel;

/// Maximum voxel value
#define VOXEL_MAX 127

/// Reserved voxel value for "UNVISITED"
#define VOXEL_UNVISITED -128

/// Voxel sampler space definition class.
class SampleSpace {
  public:
    /// Sanity check of the sample space parameters.
    inline bool IsValid()
    {
      return ((mDiv[0] >= 1) && (mDiv[1] >= 1) && (mDiv[2] >= 1) &&
              (mAABB.mMax.x > mAABB.mMin.x) && (mAABB.mMax.y > mAABB.mMin.y) &&
              (mAABB.mMax.z > mAABB.mMin.z));
    }

    /// Return the voxel spacing (voxel size).
    inline Vector3 VoxelSize()
    {
      Vector3 d = mAABB.Diagonal();
      d.x /= mDiv[0];
      d.y /= mDiv[1];
      d.z /= mDiv[2];
      return d;
    }

    /// Define the sample space. This method calculates the optimal sample
    /// space parameters, based on the given minimal enclosing bounding box
    /// and the desired resolution.
    void DefineSpace(BoundingBox &aAABB, Vector3 aResolution);

    /// Define the sample space (uniform resolution).
    void DefineSpace(BoundingBox &aAABB, double aResolution);

    BoundingBox mAABB; ///< Bonding box for the voxel space.
    int mDiv[3];       ///< Number of divisions of the voxel space (x, y, z).
};

// Quick slice filler function (like memset).
void FillSlice(Voxel * aSlice, Voxel aValue, int aCount);

}

#endif // _SAMPLESPACE_H_
