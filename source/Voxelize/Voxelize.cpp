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

#include "Voxelize.h"
#include <stdexcept>

using namespace std;

namespace csg {

//-----------------------------------------------------------------------------
// Voxelize
//-----------------------------------------------------------------------------

Voxelize::Voxelize()
{
  mVoxelSpaceDefined = false;
}

void Voxelize::SetVoxelSpace(float aMinX, float aMinY, float aMinZ, float aMaxX,
  float aMaxY, float aMaxZ, int aDivX, int aDivY, int aDivZ)
{
  // Sanity check: are these valid parameters?
  if((aDivX < 1) || (aDivY < 1) || (aDivZ < 1) ||
     ((aMaxX - aMinX) <= 0.0) || ((aMaxY - aMinY) <= 0.0) ||
     ((aMaxZ - aMinZ) <= 0.0))
    throw runtime_error("Invalid voxel space dimensions.");

  // Store voxel space dimensions
  mMin = Vector3(aMinX, aMinY, aMinZ);
  mMax = Vector3(aMaxX, aMaxY, aMaxZ);
  mDiv[0] = aDivX;
  mDiv[1] = aDivY;
  mDiv[2] = aDivZ;

  // The voxel space is now defined
  mVoxelSpaceDefined = true;
}

}
