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

#include "Voxelize.h"
#include <stdexcept>
#include <cmath>

using namespace std;

namespace csg {

//-----------------------------------------------------------------------------
// SampleSpace
//-----------------------------------------------------------------------------

void SampleSpace::DefineSpace(BoundingBox &aAABB, Vector3 aResolution)
{
  // Sanity check
  if((aResolution.x < 1e-50) ||(aResolution.y < 1e-50) ||
     (aResolution.z < 1e-50))
    throw runtime_error("Invalid resolution.");

  // Determine nominal division
  Vector3 d = aAABB.Diagonal();
  mDiv[0] = int(ceil(d.x / aResolution.x));
  mDiv[1] = int(ceil(d.y / aResolution.y));
  mDiv[2] = int(ceil(d.z / aResolution.z));
  for(int i = 0; i < 3; ++ i)
    if(mDiv[i] < 1)
      mDiv[i] = 1;

  // Compensate AABB size for boundary gradients etc.
  for(int i = 0; i < 3; ++ i)
    mDiv[i] += 4;

  // Calculate resulting bounding box
  Vector3 center = (aAABB.mMin + aAABB.mMax) * 0.5;
  Vector3 radius = Vector3(mDiv[0] * aResolution.x,
                           mDiv[1] * aResolution.y,
                           mDiv[2] * aResolution.z) * 0.5;
  mAABB.mMin = center - radius;
  mAABB.mMax = center + radius;
}

}
