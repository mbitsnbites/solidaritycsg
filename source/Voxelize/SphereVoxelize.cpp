/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "SphereVoxelize.h"
#include <stdexcept>

using namespace std;

namespace csg {

//-----------------------------------------------------------------------------
// SphereVoxelize
//-----------------------------------------------------------------------------

SphereVoxelize::SphereVoxelize()
{
  mVoxelSpaceDefined = false;
}

SphereVoxelize::~SphereVoxelize()
{
}

void SphereVoxelize::SetSphere(Vector3 aCenter, double aRadius)
{
  mCenter = aCenter;
  mRadius = aRadius;
}

void SphereVoxelize::CalculateSlice(Voxel * aSlice, int aZ)
{
  // Check that the voxel space has been properly set up
  if(!mVoxelSpaceDefined)
    throw runtime_error("Undefined voxel space dimensions.");

  // Calculate voxel size (diagonal of a voxel box)
  double dx = (mMax.x - mMin.x) / mDiv[0];
  double dy = (mMax.y - mMin.y) / mDiv[1];
  double dz = (mMax.z - mMin.z) / mDiv[2];
  double voxelSizeInv = 1.0 / sqrt(dx * dx + dy * dy + dz * dz);

  // Generate slice
  Voxel * vPtr = aSlice;
  Vector3 p;
  p.z = mMin.z + dz * aZ;
  for(int i = 0; i < mDiv[0]; ++ i)
  {
    p.y = mMin.y + dy * i;
    for(int j = 0; j < mDiv[0]; ++ j)
    {
      p.x = mMin.x + dx * j;

      // Calculate distance from point to the shape surface
      double dist = (mRadius - (p - mCenter).Abs()) * voxelSizeInv;

      // Convert distance to a voxel value
      Voxel v;
      if(dist <= -1.0)
        v = -VOXEL_MAX;
      else if(dist >= 1.0)
        v = VOXEL_MAX;
      else
        v = (Voxel)(dist * VOXEL_MAX + 0.5);
      *vPtr ++ = v;
    }
  }
}

}
