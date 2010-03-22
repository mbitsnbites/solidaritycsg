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

#include "SphereVoxelize.h"
#include <stdexcept>

using namespace std;

namespace csg {

//-----------------------------------------------------------------------------
// SphereVoxelize
//-----------------------------------------------------------------------------

void SphereVoxelize::SetSphere(Vector3 aCenter, double aRadius)
{
  // Collect sphere parameters
  mCenter = aCenter;
  mRadius = aRadius;

  // Update the shape bounding box
  Vector3 r = Vector3(mRadius, mRadius, mRadius);
  mAABB.mMin = mCenter - r;
  mAABB.mMax = mCenter + r;
}

bool SphereVoxelize::CalculateSlice(Voxel * aSlice, int aZ, int &aMinX,
  int &aMinY, int &aMaxX, int &aMaxY)
{
  // Check that the voxel space has been properly set up
  if(!mSampleSpace || !mSampleSpace->IsValid())
    throw runtime_error("Undefined/invalid voxel space dimensions.");

  // Calculate voxel size (diagonal of a voxel box)
  Vector3 d = mSampleSpace->VoxelSize();
  double voxelSizeInv = d.Abs();
  if(voxelSizeInv < 1e-50)
    throw runtime_error("Invalid voxel space dimensions.");
  voxelSizeInv = 1.0 / d.Abs();

  // Determine bounding rectangle
  aMinX = aMinY = 0;
  aMaxX = mSampleSpace->mDiv[0] - 1;
  aMaxY = mSampleSpace->mDiv[1] - 1;

  // Generate slice
  Voxel * vPtr = aSlice;
  Vector3 p;
  p.z = mSampleSpace->mAABB.mMin.z + d.z * aZ;
  for(int i = 0; i < mSampleSpace->mDiv[1]; ++ i)
  {
    p.y = mSampleSpace->mAABB.mMin.y + d.y * i;
    for(int j = 0; j < mSampleSpace->mDiv[0]; ++ j)
    {
      p.x = mSampleSpace->mAABB.mMin.x + d.x * j;

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

  return true;
}

}
