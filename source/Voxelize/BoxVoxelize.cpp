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

#include "BoxVoxelize.h"
#include <stdexcept>

using namespace std;

namespace csg {

//-----------------------------------------------------------------------------
// BoxVoxelize
//-----------------------------------------------------------------------------

void BoxVoxelize::SetBox(Vector3 aCenter, Vector3 aSides)
{
  // Collect box parameters
  mCenter = aCenter;
  mSides = aSides;

  // Update the shape bounding box
  mAABB.mMin = mCenter - mSides * 0.5;
  mAABB.mMax = mCenter + mSides * 0.5;
}

bool BoxVoxelize::CalculateSlice(Voxel * aSlice, int aZ, int &aMinX, int &aMinY,
  int &aMaxX, int &aMaxY)
{
  // Check that the voxel space has been properly set up
  if(!mSampleSpace || !mSampleSpace->IsValid())
    throw runtime_error("Undefined/invalid voxel space dimensions.");

  // Calculate voxel size (diagonal of a voxel box)
  Vector3 d = mSampleSpace->VoxelSize();
  if((d.x < 1e-50) || (d.y < 1e-50) || (d.z < 1e-50))
    throw runtime_error("Invalid voxel space dimensions.");
  Vector3 dInv(0.5 / d.x, 0.5 / d.y, 0.5 / d.z);

  // Determine bounding rectangle
  aMinX = aMinY = 0;
  aMaxX = mSampleSpace->mDiv[0] - 1;
  aMaxY = mSampleSpace->mDiv[1] - 1;

  // Generate slice
  Voxel * vPtr = aSlice;
  Vector3 p;
  Vector3 dist;
  p.z = mSampleSpace->mAABB.mMin.z + d.z * aZ;

  // Calculate Z-distance from point to the shape surface
  if(p.z < mAABB.mMin.z)
    dist.z = -(mAABB.mMin.z - p.z);
  else if(p.z > mAABB.mMax.z)
    dist.z = -(p.z - mAABB.mMax.z);
  else if(p.z < mCenter.z)
    dist.z = p.z - mAABB.mMin.z;
  else
    dist.z = mAABB.mMax.z - p.z;
  dist.z *= dInv.z;

  for(int i = 0; i < mSampleSpace->mDiv[1]; ++ i)
  {
    p.y = mSampleSpace->mAABB.mMin.y + d.y * i;

    // Calculate Y-distance from point to the shape surface
    if(p.y < mAABB.mMin.y)
      dist.y = -(mAABB.mMin.y - p.y);
    else if(p.y > mAABB.mMax.y)
      dist.y = -(p.y - mAABB.mMax.y);
    else if(p.y < mCenter.y)
      dist.y = p.y - mAABB.mMin.y;
    else
      dist.y = mAABB.mMax.y - p.y;
    dist.y *= dInv.y;

    for(int j = 0; j < mSampleSpace->mDiv[0]; ++ j)
    {
      p.x = mSampleSpace->mAABB.mMin.x + d.x * j;

      // Calculate X-distance from point to the shape surface
      if(p.x < mAABB.mMin.x)
        dist.x = -(mAABB.mMin.x - p.x);
      else if(p.x > mAABB.mMax.x)
        dist.x = -(p.x - mAABB.mMax.x);
      else if(p.x < mCenter.x)
        dist.x = p.x - mAABB.mMin.x;
      else
        dist.x = mAABB.mMax.x - p.x;
      dist.x *= dInv.x;

      // Determine in/out
      double absDist;
      if((dist.x > 0) && (dist.y > 0) && (dist.z > 0))
        absDist = dist.Abs();
      else
      {
        absDist = 0;
        if(dist.x < 0)
          absDist += dist.x * dist.x;
        if(dist.y < 0)
          absDist += dist.y * dist.y;
        if(dist.z < 0)
          absDist += dist.z * dist.z;
        absDist = -sqrt(absDist);
      }

      // Convert distance to a voxel value
      Voxel v;
      if(absDist <= -1.0)
        v = -VOXEL_MAX;
      else if(absDist >= 1.0)
        v = VOXEL_MAX;
      else
        v = (Voxel)(absDist * VOXEL_MAX + 0.5);
      *vPtr ++ = v;
    }
  }

  return true;
}

}
